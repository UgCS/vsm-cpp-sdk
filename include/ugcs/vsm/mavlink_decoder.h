// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file mavlink_decoder.h
 *
 * Mavlink decoder
 */
#include <ugcs/vsm/callback.h>
#include <ugcs/vsm/mavlink.h>

#include <unordered_map>

#ifndef _UGCS_VSM_MAVLINK_DECODER_H_
#define _UGCS_VSM_MAVLINK_DECODER_H_

namespace ugcs {
namespace vsm {

/** Decodes Mavlink 1.0 messages from byte stream. */
class Mavlink_decoder {
private:
    /** Decoding state. */
    enum class State {
        /** Waiting for start sign. */
        STX,
        /** Parsing mavlink1. */
        VER1,
        /** Parsing mavlink2. */
        VER2
    };

public:
    /** Handler type of the received Mavlink message. Arguments are:
     * - Payload buffer
     * - Message id
     * - Sending system id
     * - Sending component id
     */
    typedef Callback_proxy<void, Io_buffer::Ptr, mavlink::MESSAGE_ID_TYPE,
        uint8_t, uint8_t, uint32_t> Handler;

    /** Handler for the raw data going through the decoder. */
    typedef Callback_proxy<void, Io_buffer::Ptr> Raw_data_handler;

    /** Convenience builder for Mavlink decoder handlers. */
    DEFINE_CALLBACK_BUILDER(
            Make_decoder_handler,
            (Io_buffer::Ptr, mavlink::MESSAGE_ID_TYPE, uint8_t, uint8_t, uint32_t),
            (nullptr, mavlink::MESSAGE_ID::DEBUG_VALUE, mavlink::SYSTEM_ID_NONE, 0, 0))

    /** Convenience builder for raw data handlers. */
    DEFINE_CALLBACK_BUILDER(Make_raw_data_handler, (Io_buffer::Ptr), (nullptr))

    /** Decoder statistics. */
    struct Stats {
        /** Messages processed by registered handler. Total and per system_id. */
        uint64_t handled = 0;
        /** Messages without handler, dropped. Total and per system_id. */
        uint64_t no_handler = 0;
        /** Messages with wrong checksum. Only total for the connection is counted. */
        uint64_t bad_checksum = 0;
        /** Messages with wrong length, but correct checksum. Total and per system_id. */
        uint64_t bad_length = 0;
        /** Messages with unknown id. Note that checksum is not verified for
         * such messages. Only total for the connection is counted.
         */
        uint64_t unknown_id = 0;
        /** Total received bytes, including any error or not handled messages. Only total for the connection is counted. */
        uint64_t bytes_received = 0;
        /** Number of STX bytes found during decoding, i.e. how many times packet
         * decode was tried to be started. Only total for the connection is counted. */
        uint64_t stx_syncs = 0;
    };


    /** Default constructor. */
    Mavlink_decoder():
        packet_buf(ugcs::vsm::Io_buffer::Create())
    {
    }

    /** Delete copy constructor. */
    Mavlink_decoder(const Mavlink_decoder&) = delete;

    /** Should be called prior to intention to delete the instance. */
    void
    Disable()
    {
        handler = Handler();
        data_handler = Raw_data_handler();
    }

    /**
     * Register handler for successfully decoded Mavlink messages.
     */
    void
    Register_handler(Handler handler)
    {
        this->handler = handler;
    }

    void
    Register_raw_data_handler(Raw_data_handler handler)
    {
        data_handler = handler;
    }

    /** Decode buffer from the wire. */
    void
    Decode(Io_buffer::Ptr buffer)
    {
        // LOG_DBG("read: %s", buffer->Get_hex().c_str());
        if (data_handler) {
            data_handler(buffer);
        }
        packet_buf = packet_buf->Concatenate(buffer);
        size_t packet_len;
        const uint8_t* data;
        next_read_len = 0;

        while (true) {
            size_t buffer_len = packet_buf->Get_length();
            {   // lock scope
                std::lock_guard<std::mutex> stats_lock(stats_mutex);
                stats[mavlink::SYSTEM_ID_ANY].bytes_received += buffer_len;
                if (state == State::STX) {
                    size_t len_skipped = 0;
                    if (buffer_len < mavlink::MAVLINK_1_MIN_FRAME_LEN) {
                        // need at least minimum frame length of data.
                        next_read_len = mavlink::MAVLINK_1_MIN_FRAME_LEN - buffer_len;
                        break;
                    }
                    // look for signature in received data.
                    data = static_cast<const uint8_t*>(packet_buf->Get_data());
                    for (; len_skipped < buffer_len; len_skipped++, data++) {
                        if (*data == mavlink::START_SIGN) {
                            // found preamble. Start receiving payload.
                            state = State::VER1;
                            stats[mavlink::SYSTEM_ID_ANY].stx_syncs++;
                            // slice off the preamble.
                            packet_buf = packet_buf->Slice(1);
                            break;
                        }
                        if (*data == mavlink::START_SIGN2) {
                            // found preamble. Start receiving payload.
                            state = State::VER2;
                            stats[mavlink::SYSTEM_ID_ANY].stx_syncs++;
                            // slice off the preamble.
                            packet_buf = packet_buf->Slice(1);
                            break;
                        }
                    }
                    if (len_skipped) {
                        // slice off the skipped bytes.
                        packet_buf = packet_buf->Slice(len_skipped);
                    }
                }
            }
            if (state == State::VER1 || state == State::VER2) {
                size_t wrapper_len; // non-payload data len excluding signature.
                if (state == State::VER1) {
                    wrapper_len = mavlink::MAVLINK_1_HEADER_LEN - 1 + 2;
                } else {
                    wrapper_len = mavlink::MAVLINK_2_HEADER_LEN - 1 + 2;
                }
                buffer_len = packet_buf->Get_length();
                if (buffer_len == 0) {
                    // need at least the minimum packet len. Initiate next read.
                    next_read_len = wrapper_len;
                    break;
                }
                data = static_cast<const uint8_t*>(packet_buf->Get_data());
                packet_len = wrapper_len + static_cast<size_t>(*data);
                if (packet_len > buffer_len) {
                    // need the whole packet. Initiate next read.
                    next_read_len = packet_len - buffer_len;
                    break;
                }
                if (Decode_packet(packet_buf)) {
                    // decoder suceeded. Slice off the decoded packet.
                    packet_buf = packet_buf->Slice(packet_len);
                }
                // if decoder failed, we restart the search for
                // next preamble in existing data otherwise
                // continue with next byte after the decoded packet.
                state = State::STX;
            }
        }
    }

    /** Get the exact number of bytes which should be read by underlying
     * I/O subsystem and fed to the decoder.
     * @return Exact number of bytes to be read by next read operation.
     */
    size_t
    Get_next_read_size() const
    {
        return next_read_len;
    }

    /** Get read-only access to statistics.
     * Supports multiple system_ids on one connection.
     * @param system_id system id to get statistics for. Use mavlink::SYSTEM_ID_ANY to get total for all system_ids.
     * @return Readonly reference to the Stats structure for given system_id.
     * */
    const Mavlink_decoder::Stats
    Get_stats(int system_id)
    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        return stats[system_id];
    }

    /** Get read-only access to common statistics. */
    const Mavlink_decoder::Stats
    Get_common_stats()
    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        return stats[mavlink::SYSTEM_ID_ANY];
    }

private:
    bool
    Decode_packet(Io_buffer::Ptr buffer)
    {
        auto data = static_cast<const uint8_t*>(buffer->Get_data());
        uint16_t payload_len = data[0];
        uint8_t system_id;
        uint8_t component_id;
        uint8_t seq;
        uint8_t header_len;
        mavlink::MESSAGE_ID_TYPE msg_id;

        if (state == State::VER2) {
            seq = data[3];
            system_id = data[4];
            component_id = data[5];
            msg_id =
                static_cast<int>(data[6]) +
                (static_cast<int>(data[7]) << 8) +
                (static_cast<int>(data[8]) << 16);
            header_len = mavlink::MAVLINK_2_HEADER_LEN - 1;
        } else {
            seq = data[1];
            system_id = data[2];
            component_id = data[3];
            msg_id = data[4];
            header_len = mavlink::MAVLINK_1_HEADER_LEN - 1;
        }

        mavlink::Checksum sum(data, header_len);

        mavlink::Extra_byte_length_pair crc_byte_len_pair;
        // Try all extensions.
        if (    !sum.Get_extra_byte_length_pair(msg_id, crc_byte_len_pair, mavlink::Extension::Get())
            &&  !sum.Get_extra_byte_length_pair(msg_id, crc_byte_len_pair, mavlink::apm::Extension::Get())
            &&  !sum.Get_extra_byte_length_pair(msg_id, crc_byte_len_pair, mavlink::sph::Extension::Get())) {
            std::lock_guard<std::mutex> stats_lock(stats_mutex);
            stats[mavlink::SYSTEM_ID_ANY].unknown_id++;
            LOG_DEBUG("Unknown Mavlink message id: %d system id: %d component id: %d)", msg_id, system_id, component_id);
            return false;
        }

        sum.Accumulate(data + header_len, payload_len);

        uint16_t sum_calc = sum.Accumulate(crc_byte_len_pair.first);
        /* Convert checksum in Mavlink byte order to a host byte order
         * compatible type. */
        auto sum_recv = reinterpret_cast<const mavlink::Uint16*>(data + header_len + payload_len);

        bool cksum_ok = sum_calc == *sum_recv;
        bool length_ok = crc_byte_len_pair.second == payload_len;

        std::unique_lock<std::mutex> stats_lock(stats_mutex);
        if (cksum_ok && (length_ok || state == State::VER2)) {
            /*
             * Fully valid packet received.
             */
            if (handler) {
                stats[system_id].handled++;
                stats[mavlink::SYSTEM_ID_ANY].handled++;
                stats_lock.unlock();
                handler(Io_buffer::Create(*buffer, header_len, payload_len), msg_id, system_id, component_id, seq);
            } else {
                stats[system_id].no_handler++;
                stats[mavlink::SYSTEM_ID_ANY].no_handler++;
                LOG_DEBUG("Mavlink message %d handler not registered.", msg_id);
            }
            return true;
        } else {
            LOG_DEBUG("Invalid Mavlink message id: %d system id: %d component id: %d)", msg_id, system_id, component_id);
            if (cksum_ok) {
                stats[system_id].bad_length++;
                stats[mavlink::SYSTEM_ID_ANY].bad_length++;
                LOG_DEBUG("Mavlink payload length mismatch, recv=%d wanted=%d.",
                    payload_len, crc_byte_len_pair.second);
            } else {
                stats[mavlink::SYSTEM_ID_ANY].bad_checksum++;
            }
            return false;
        }
    }


    /** Current decoder state. */
    State state = State::STX;

    /** Handler for decoded messages. */
    Handler handler;

    /** Raw data handler. */
    Raw_data_handler data_handler;

    /** Statistics. */
    std::unordered_map<int, Stats> stats;
    std::mutex stats_mutex;

    /** Packet buffer. */
    ugcs::vsm::Io_buffer::Ptr packet_buf;

    size_t next_read_len = mavlink::MAVLINK_1_MIN_FRAME_LEN;
};

} /* namespace vsm */
} /* namespace ugcs */

#endif /* _UGCS_VSM_MAVLINK_DECODER_H_ */
