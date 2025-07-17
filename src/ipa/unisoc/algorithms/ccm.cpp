/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Unisoc ISP color correction matrix
 */

#include "ccm.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/fixedpoint.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

LOG_DEFINE_CATEGORY(UnisocCcm)

Ccm::Ccm()
{
}

int Ccm::init([[maybe_unused]] IPAContext &context,
	      const YamlObject &tuningData)
{
	int ret = ccm_.readYaml(tuningData["ccms"], "ct", "ccm");
	if (ret < 0) {
		LOG(UnisocCcm, Error)
			<< "Failed to parse CCM from tuning file.";
		return ret;
	}

	return 0;
}

void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, UnisocParams *params)
{
	if (frame != 0)
		return;

	Matrix<float, 3, 3> ccm = ccm_.getInterpolated(context.activeState.awb.temperatureK);

	context.activeState.ccm.ccm = ccm;
	frameContext.ccm.ccm = ccm;

	auto ccmCfg = params->ispBlock<BlockType::Ccm>();

	ccmCfg->r_r = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[0][0]);
	ccmCfg->r_g = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[0][1]);
	ccmCfg->r_b = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[0][2]);
	ccmCfg->g_r = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[1][0]);
	ccmCfg->g_g = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[1][1]);
	ccmCfg->g_b = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[1][2]);
	ccmCfg->b_r = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[2][0]);
	ccmCfg->b_g = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[2][1]);
	ccmCfg->b_b = floatingToFixedPoint<4, 10, uint16_t, float>(ccm[2][2]);
}

void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const sprd_dcam_statistics *stats,
		  ControlList &metadata)
{
	float m[9];
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			m[i * 3 + j] = frameContext.ccm.ccm[i][j];
	}
	metadata.set(controls::ColourCorrectionMatrix, m);
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
