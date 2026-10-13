
/**
 * @file def.h
 * @brief class definitions used by other headers
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "tensor.h"

namespace sharpa {
namespace tactile {

/**
 * @brief base class of Logger, for user to implement custom Loggers 
 */
class LoggerBase {
public:
    /**
     * @brief for logging debug messages
     * @param msg message
     */
    virtual void debug(const std::string &msg) = 0;
    /**
     * @brief for logging info messages
     * @param msg message
     */
    virtual void info(const std::string &msg) = 0;
    /**
     * @brief for logging warn messages
     * @param msg message
     */
    virtual void warn(const std::string &msg) = 0;
    /**
     * @brief for logging error messages
     * @param msg message
     */
    virtual void error(const std::string &msg) = 0;

    /**
     * @brief for destructing LoggerBase
     */
    virtual ~LoggerBase() = default;
};

/**
 * @brief tactile data frame
 */
struct Frame {
    /** pointer of data frame */
    using Ptr = std::shared_ptr<Frame>;
    /** frame id, unique and increasing for each tactile sensor
     * (reset by next hardware restart)
     * (as 32-bit integer, it may overflow)
     */
    int frame_id;
    /** channel, see Touch constructor */
    int channel;
    /** time stamp */
    double ts;
    /**
     * contents
     * key can be ["RAW", "DEFORM", "F6", ...]
     */
    std::map<std::string, DataBlock::Ptr> content;
    bool is_to_infer() const;
    /**
     * copy a frame
     */
    Frame::Ptr copy() const;
};

enum class TACTILE_TAG{
    WAVE_HA4=0,
    WAVE_HB1=1,
};

enum class TACTILE_ERROR_CODE {
    NO_ERROR = 0,
    HOST_IP_INVALID = 1,
    HOST_PORT_OCCUPIED = 2,
    HOST_LISTEN_FAULT = 3,
    TACCTILE_SDK_ALREADY_RUN = 4,
    TCP_CONNECT_FAILED = 6,
    ERROR_CODE_RESERVED = 5,
    TCP_SYSTEM_ERROR = 7,
    TCP_RECV_ERROR = 8,
    TCP_SEND_ERROR = 9,
    TCP_MSG_PARSE_FALSE = 10,
    CMD_EXEC_ERROR = 11,
    BOARD_CFG_KEY_NOT_EXIST = 12,
    BOARD_IP_NOT_CONFIGURED = 13,
    BOARD_CHANNEL_CONFLICT = 14,
    UNKNOWN_ERROR = 100,
};
}
}
