#include "SharpaWaveSDK.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <sstream>

// Joint names (indices 0..21)
const std::vector<std::string> JOINT_NAMES = {
    "Thumb CMC Flexion/Extension",
    "Thumb CMC Abduction/Adduction", 
    "Thumb MCP Flexion/Extension",
    "Thumb MCP Abduction/Adduction",
    "Thumb DIP Flexion/Extension",
    "Index MCP Flexion/Extension",
    "Index MCP Abduction/Adduction",
    "Index PIP Flexion/Extension", 
    "Index DIP Flexion/Extension",
    "Middle MCP Flexion/Extension",
    "Middle MCP Abduction/Adduction",
    "Middle PIP Flexion/Extension",
    "Middle DIP Flexion/Extension", 
    "Ring MCP Flexion/Extension",
    "Ring MCP Abduction/Adduction",
    "Ring PIP Flexion/Extension",
    "Ring DIP Flexion/Extension",
    "Pinky CMC Flexion/Extension",
    "Pinky MCP Flexion/Extension",
    "Pinky MCP Abduction/Adduction", 
    "Pinky PIP Flexion/Extension",
    "Pinky DIP Flexion/Extension",
};

// Angle ranges (in degrees) - pairs of (min, max)
const std::vector<std::pair<float, float>> ANGLE_RANGES = {
    {0.0f, 50.0f},   // Thumb CMC Flexion/Extension
    {0.0f, 10.0f},   // Thumb CMC Abduction/Adduction
    {0.0f, 30.0f},   // Thumb MCP Flexion/Extension
    {0.0f, 10.0f},   // Thumb MCP Abduction/Adduction
    {0.0f, 40.0f},   // Thumb DIP Flexion/Extension
    {0.0f, 20.0f},   // Index MCP Flexion/Extension
    {-20.0f, 20.0f}, // Index MCP Abduction/Adduction
    {0.0f, 20.0f},   // Index PIP Flexion/Extension
    {0.0f, 20.0f},   // Index DIP Flexion/Extension
    {0.0f, 20.0f},   // Middle MCP Flexion/Extension
    {-20.0f, 20.0f}, // Middle MCP Abduction/Adduction
    {0.0f, 20.0f},   // Middle PIP Flexion/Extension
    {0.0f, 20.0f},   // Middle DIP Flexion/Extension
    {0.0f, 20.0f},   // Ring MCP Flexion/Extension
    {-20.0f, 20.0f}, // Ring MCP Abduction/Adduction
    {0.0f, 20.0f},   // Ring PIP Flexion/Extension
    {0.0f, 20.0f},   // Ring DIP Flexion/Extension
    {0.0f, 10.0f},   // Pinky CMC Flexion/Extension
    {0.0f, 20.0f},   // Pinky MCP Flexion/Extension
    {-20.0f, 20.0f}, // Pinky MCP Abduction/Adduction
    {0.0f, 20.0f},   // Pinky PIP Flexion/Extension
    {0.0f, 20.0f},   // Pinky DIP Flexion/Extension
};

// Automatically detect device and return device
sharpa::SharpaWave* auto_detect_hand()
{
    std::cout << "Searching for devices..." << std::endl;
    
    try {
        auto& manager = sharpa::SharpaWaveManager::get_instance();
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for 1 second for device discovery to complete
        
        while (true) {
            auto devices = manager.get_all_device_sn();
            if (devices.empty()) {
                std::cout << "No available devices found" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            } else {
                std::cout << "Device found: " << devices[0] << std::endl;
                return &manager.connect(devices[0]);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to connect to device: " << e.what() << std::endl;
        exit(1);
    }
}

// Initialize hand settings
bool initialize(sharpa::SharpaWave* hand)
{
    auto error = hand->set_control_mode(sharpa::ControlMode::POSITION);
    if (error.code != 0) {
        std::cerr << "Failed to set control mode: " << error.message << std::endl;
        return false;
    }
    
    error = hand->set_speed_coeff(0.3f);
    if (error.code != 0) {
        std::cerr << "Failed to set speed coeff: " << error.message << std::endl;
        return false;
    }
    
    error = hand->set_current_coeff(0.6f);
    if (error.code != 0) {
        std::cerr << "Failed to set current coeff: " << error.message << std::endl;
        return false;
    }
    
    error = hand->set_control_source(sharpa::ControlSource::SDK);
    if (error.code != 0) {
        std::cerr << "Failed to set control source: " << error.message << std::endl;
        return false;
    }
    
    return true;
}

// Run cycles with sinusoidal motion
void run_cycles(sharpa::SharpaWave* hand)
{
    const float cycle_duration_s = 1.0f;
    const float dt = 0.01f;
    const int total_tick = static_cast<int>(cycle_duration_s / dt);
    const int cycles = 3;
    std::vector<float> angle_list(22, 0.0f);
    
    // Index of MCP joints to skip
    const std::vector<int> SKIP_MOVE_JOINT_INDEX = {6, 10, 14, 17, 19};
    
    for (int cycle = 0; cycle < cycles; ++cycle) {
        for (int tick = 0; tick < total_tick; ++tick) {
            for (int i = 0; i < 22; ++i) {
                // Check if this joint should be skipped
                bool should_skip = false;
                for (int skip_idx : SKIP_MOVE_JOINT_INDEX) {
                    if (i == skip_idx) {
                        should_skip = true;
                        break;
                    }
                }
                if (should_skip) {
                    continue;
                }
                
                // Calculate target angle using sinusoidal motion
                float lo = ANGLE_RANGES[i].first;
                float hi = ANGLE_RANGES[i].second;
                float target_angle_deg = (lo + hi) / 2.0f + (hi - lo) / 2.0f * 
                                         std::sin(2.0f * M_PI * tick / total_tick);
                angle_list[i] = target_angle_deg * M_PI / 180.0f; // Convert to radians
            }
            
            auto error = hand->set_joint_position(angle_list, true);
            if (error.code != 0) {
                std::cerr << "Failed to set joint position(rad): " << error.message << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::duration<float>(dt));
        }
        
        // Get and print joint positions after each cycle
        auto [error, angles] = hand->get_joint_position_degree();
        if (error.code != 0) {
            std::cerr << "Failed to get joint position(degree): " << error.message << std::endl;
        } else {
            std::ostringstream oss;
            for (size_t i = 0; i < angles.size(); ++i) {
                if (i > 0) oss << ",";
                oss << std::fixed << std::setprecision(2) << angles[i];
            }
            std::cout << "angles " << oss.str() << std::endl;
        }
    }
}

int main()
{
    std::cout << "Sharpa Wave Example - Starting" << std::endl;
    
    // Try to automatically detect device
    sharpa::SharpaWave* hand = auto_detect_hand();
    if (hand == nullptr) {
        std::cerr << "Error: No available device found" << std::endl;
        return 1;
    }
    
    std::cout << "Sharpa Wave Example - Init Hand Running Mode" << std::endl;
    if (!initialize(hand)) {
        std::cerr << "Error: Failed to initialize hand" << std::endl;
        return 1;
    }
    
    std::cout << "Sharpa Wave Example - Run Demo Gestures" << std::endl;
    hand->start();
    
    // Reset hand to init position with interpolation mode (move smoothly)
    bool enable_interpolation_mode = true;
    std::vector<float> init_angles(22, 0.0f);
    hand->set_joint_position(init_angles, enable_interpolation_mode);
    
    run_cycles(hand);
    
    // Reset to init position again
    hand->set_joint_position(init_angles, enable_interpolation_mode);
    
    std::cout << "Sharpa Wave Example - Stop Hand Running Mode" << std::endl;
    hand->stop();
    sharpa::SharpaWaveManager::get_instance().disconnect_all();
    std::cout << "Sharpa Wave Example - Stopped" << std::endl;
    
    return 0;
}
