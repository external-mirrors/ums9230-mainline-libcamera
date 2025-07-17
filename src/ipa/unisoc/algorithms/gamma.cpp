/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Unisoc ISP gamma table configuration
 */

#include "gamma.h"

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::unisoc::algorithms {

LOG_DEFINE_CATEGORY(UnisocGamma)

static constexpr unsigned int kGammaCurveSize = 257;

Gamma::Gamma()
{
}

int Gamma::init([[maybe_unused]] IPAContext &context,
		const YamlObject &tuningData)
{
	curveR_ = tuningData["red"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (curveR_.size() != kGammaCurveSize) {
		LOG(UnisocGamma, Error)
			<< "Invalid red gamma curve: expected "
			<< kGammaCurveSize
			<< " elements, got " << curveR_.size();
		return -EINVAL;
	}

	curveG_ = tuningData["green"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (curveG_.size() != kGammaCurveSize) {
		LOG(UnisocGamma, Error)
			<< "Invalid green gamma curve: expected "
			<< kGammaCurveSize
			<< " elements, got " << curveR_.size();
		return -EINVAL;
	}

	curveB_ = tuningData["blue"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (curveB_.size() != kGammaCurveSize) {
		LOG(UnisocGamma, Error)
			<< "Invalid blue gamma curve: expected "
			<< kGammaCurveSize
			<< " elements, got " << curveR_.size();
		return -EINVAL;
	}

	return 0;
}

void Gamma::prepare([[maybe_unused]] IPAContext &context,
		    [[maybe_unused]] const uint32_t frame,
		    [[maybe_unused]] IPAFrameContext &frameContext,
		    UnisocParams *params)
{
	if (frame != 0)
		return;

	auto gammaCfg = params->ispBlock<BlockType::Gamma>();

	for (unsigned int i = 0; i < kGammaCurveSize; i++) {
		gammaCfg->r_tbl[i] = curveR_[i];
		gammaCfg->g_tbl[i] = curveG_[i];
		gammaCfg->b_tbl[i] = curveB_[i];
	}
}

REGISTER_IPA_ALGORITHM(Gamma, "Gamma")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
