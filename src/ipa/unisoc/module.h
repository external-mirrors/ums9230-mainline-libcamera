/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * module.h - Unisoc IPA Module
 */

#pragma once

#include <linux/media/sprd/camsys-config.h>

#include <libcamera/ipa/unisoc_ipa_interface.h>

#include <libipa/module.h>

#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::unisoc {

using Module = ipa::Module<IPAContext, IPAFrameContext, IPACameraSensorInfo,
			   UnisocParams, sprd_dcam_statistics>;

} /* namespace ipa::unisoc */

} /* namespace libcamera*/
