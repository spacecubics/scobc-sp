# Copyright (c) 2026 Space Cubics Inc.
# SPDX-License-Identifier: Apache-2.0

target_sources_ifdef(CONFIG_I2C_SC app PRIVATE i2c_sc.c)
