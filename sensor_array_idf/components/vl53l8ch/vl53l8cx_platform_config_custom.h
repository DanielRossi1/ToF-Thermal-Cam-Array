#ifndef _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_
#define _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_

// Retrieve all 4 targets per zone for maximum data richness.
#define VL53L8CX_NB_TARGET_PER_ZONE  4U

// Disable optional outputs that cost I2C bandwidth but are not
// used in the packet (motion indicator is visualised separately).
// Leave everything enabled so the Python side can choose.

#endif  // _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_
