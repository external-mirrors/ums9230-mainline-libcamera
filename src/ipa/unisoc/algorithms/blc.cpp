/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Black level correction for Unisoc DCAM pipeline
 */

#include "blc.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::unisoc::algorithms {

LOG_DEFINE_CATEGORY(UnisocBlc)

Blc::Blc()
{
}

void Blc::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  UnisocParams *params)
{
	auto blcCfg = params->dcamBlock<BlockType::Blc>();

	uint16_t blackLevel = context.configuration.sensor.blackLevel >> 2;

	blcCfg->r = blackLevel;
	blcCfg->gr = blackLevel;
	blcCfg->gb = blackLevel;
	blcCfg->b = blackLevel;
}

void Blc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  [[maybe_unused]] const sprd_dcam_statistics *stats,
		  ControlList &metadata)
{
	uint16_t val = context.configuration.sensor.blackLevel;

	metadata.set(controls::SensorBlackLevels, { val, val, val, val });
}

REGISTER_IPA_ALGORITHM(Blc, "Blc")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
