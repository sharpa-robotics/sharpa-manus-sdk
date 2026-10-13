#pragma once
#include <memory>
#include <string>
#include "def.h"

namespace sharpa {
namespace tactile {

class InferOffline {
public:
    InferOffline(std::string model_path);
    ~InferOffline();
    Frame::Ptr infer(uint8_t* initial_image, uint8_t* realtime_image, int size);

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace tactile
}  // namespace sharpa
