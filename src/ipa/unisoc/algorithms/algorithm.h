/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * algorithm.h - Unisoc control algorithm interface
 */

#pragma once

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::unisoc {

class Algorithm : public libcamera::ipa::Algorithm<Module>
{
};

} /* namespace ipa::unisoc */

} /* namespace libcamera */
