
/**
 * @file touch.h
 * @brief main api of tactile_sdk
 */

#pragma once

#include <map>
#include <vector>
#include <string>
#include <functional>
#include <optional>
#include <memory>
#include <any>

#include "def.h"
#include "tensor.h"

namespace sharpa {
namespace tactile {

/**
 * @brief tactile related settings
 */
struct TouchSetting {
    /** path of neural network models
     *  e.g. {
     *    {{"model/a.onnx", std::vector{0, 1}},
     *    {{"model/b.onnx", std::vector{3}},
     *  }
     *  means a.onnx is used to infer data from channel 0 and 1, meanwhile b.onnx is used to infer data from channel 3
     */
    std::map<std::string, std::vector<int>> model_path;

    /** allowed udp packet loss,
     *  e.g. if set to 2, then frame will still be generated with less then or equal to 2 non-critical packet losses
     */
    int allowed_pack_loss{0};

    /**
     * number of frames preserved in queue
     * e.g, buffer size 1 means there is only 1 frame in queue, ensures fetched data are always latest
     */
    int buffer_size{2};

    /**
     * number of frames to be produced together as a batch
     * e.g. if set to 5, it waits for 5 frames and process them altogether
     */
    int batch_size{1};

    /**
     * number of threads used for processing
     */
    int num_worker{3};

    /**
     * numebr of frames per second, sent from one tactile sensor
     */
    int fps{30};

    /**
     * if inference should be done from device
     */
    bool infer_from_device{true};


    /**
     * if raw jpeg should be transferred from device
     */
    bool require_jpeg{true};

    /**
     * if jpeg image should be decoded
     */
    bool decode_jpeg{true};

    /**
     * if firmware should be updated. 
     * if set to true, then firmware will be updated when Tactile sensor is connected. .
     */
    bool update_firmware{false};

    /**
     * path of config files, e.g. thumb_map_files, etc.
     */
    std::string config_dir{""};
};

class Touch {
public:
    /**
     * @brief touch constructor
     * @param host_ip ip adress of host machine
     * @param host_port port of host machine
     * @param channels integer represent data channels:
     *   0 - 4: right hand litter finger to thumb
     *   5 - 9: left hand litter finger to thumb
     * @param board_ip ip adresses of Tactile sensor 
     * @param setting see struct TouchSetting, if not given, all deault values of struct TouchSetting will be used
     * @param logger user custom logger, see class LoggerBase 
     */
    Touch(std::string host_ip,
          int host_port,
          std::optional<std::vector<int>> channels = {},
          std::vector<std::string> board_ip = {},
          std::optional<TouchSetting> setting = {},
          std::shared_ptr<LoggerBase> logger = {});

    /**
     * @brief check if certain tactile server (specified by ip address) is ready
     * @param board_ip ip address of tactile server, i.e. "192.168.1.22"
     * @return if server is ready
     */
    bool is_ready(const std::vector<std::string> &board_ip);

    /**
     * @brief set custom callback function, which will be called once a Frame is produced
     * @param callback callback function, which takes a pointer of Frame
     */
    void set_callback(std::function<void(Frame::Ptr)> callback);

    /**
     * @brief start to receive tactile frames
     * @return 0 successfully started
     * @return 1 host ip invalid
     * @return 2 host port occupied
     * @return 3 host listen fault
     * @return 4 sdk already run
     */
    int start();

    /**
     * @brief stop to receive tactile frames
     * @return if successfully stopped
     */
    bool stop();

    /**
     * @brief check if tactile_sdk is started and running
     * @return if tactile_sdk is started and running
     */
    bool is_sdk_running();

    /**
     * @brief fetch one tactile frame
     * @param channel channel to fetch, channel definition see Touch constructor
     * @param timeout maximum time to wait(in seconds)
     * @return a pointer of frame, or nullptr if timeout
     */
    Frame::Ptr fetch(int channel, double timeout=-1);

    /**
     * @brief getter
     * @return json string of data receiving staus
     */
    std::string summary();

    /**
     * @brief reset zero state of all tactile sensors(tare sensors)
     * @param num_frames number of frames used to reset zero state in an attempt
     * @param max_retry max retry times of reseting zero state
     * @return if successfully
     */
    bool calib_zero(int num_frames = 20, int max_retry = 10);

    /**
     * @brief config parameters of tactile sensor
     * @param ip ip address of tactile sensor
     * @param channel channel of tactile sensor, -1 means all channels, channel definition see Touch constructor
     * @param key specify which parameter to modify
     * @param value value to be set. Empty means to read 
     * @param timeout maximum time to wait(in seconds), in case server doesn't response
     * @return pairs of {channel, value}
     */
    std::map<int, std::any> board_cfg(
        const std::string &ip
        , int channel
        , const std::string &key
        , std::any value
        , double timeout
    );

    /**
     * @brief update firmware of tactile sensor
     * @param ip ip address of tactile sensor
     * @param firmware_path path of fireware file
     * @return if successfully
     */
    bool board_update(
        const std::string &ip
        , const std::string &firmware_path
    );

    /**
     * @brief get 3D point location on finger surface, given image coordinates on deform image 
     * @param wave_tag wave tag of tactile sensor (see TACTILE_TAG)
     * @param channel channel of finger
     * @param row row index of pixel on deform image
     * @param col column index of pixel on deform image
     * @return x y z nx ny nz(in finger coordinate system)
     *         nullopt if there is no corresponding 3D point or channel is invalid
     */
    std::optional<std::array<float, 6>> deform_map_uv(TACTILE_TAG wave_tag, int channel, size_t row, size_t col);

    /**
     * @brief map uint8 deform value into float32 value
     * @param value_ui8 deform value in uint8 format
     * @return deform value in float32 format (unit: mm)
     */
    float deform_map_value(uint8_t value_ui8);

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}
}
