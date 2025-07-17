/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Helper for constructing Unisoc ISP parameter buffers.
 * Based on the RkISP1 parameter helpers.
 */

#pragma once

#include <linux/media/sprd/camsys-config.h>

#include <libipa/v4l2_params.h>

namespace libcamera {

namespace ipa::unisoc {

enum class BlockType {
	Ae,
	Af,
	Awb,
	Blc,
	Ccm,
	Gamma,
	Lsc,
};

namespace dcam_details {

template<BlockType B>
struct block_type {
};

#define DEFINE_DCAM_BLOCK_TYPE(id, cfgType, blkType)			\
template<>								\
struct block_type<BlockType::id> {					\
	using type = struct sprd_camsys_##cfgType##_config;		\
	static constexpr sprd_dcam_block_type blockType =		\
		sprd_dcam_block_type::SPRD_DCAM_BLOCK_##blkType;	\
};

DEFINE_DCAM_BLOCK_TYPE(Ae, ae, AE)
DEFINE_DCAM_BLOCK_TYPE(Af, af, AF)
DEFINE_DCAM_BLOCK_TYPE(Awb, awb, AWB)
DEFINE_DCAM_BLOCK_TYPE(Blc, blc, BLC)
DEFINE_DCAM_BLOCK_TYPE(Lsc, lsc, LSC)

struct param_traits {
	using id_type = BlockType;

	template<id_type Id>
	using id_to_details = block_type<Id>;
};

} /* namespace dcam_details */

namespace isp_details {

template<BlockType B>
struct block_type {
};

#define DEFINE_ISP_BLOCK_TYPE(id, cfgType, blkType)			\
template<>								\
struct block_type<BlockType::id> {					\
	using type = struct sprd_camsys_##cfgType##_config;		\
	static constexpr sprd_isp_block_type blockType =		\
		sprd_isp_block_type::SPRD_ISP_BLOCK_##blkType;		\
};

DEFINE_ISP_BLOCK_TYPE(Ccm, ccm, CCM)
DEFINE_ISP_BLOCK_TYPE(Gamma, gamma, GAMMA)

struct param_traits {
	using id_type = BlockType;

	template<id_type Id>
	using id_to_details = block_type<Id>;
};

} /* namespace isp_details */

class UnisocParams
{
public:
	UnisocParams(Span<uint8_t> dcamCfg, Span<uint8_t> ispCfg,
		     Span<uint8_t> lscGrid)
		: dcamCfg_(dcamCfg, V4L2_ISP_PARAMS_VERSION_V1)
		, ispCfg_(ispCfg, V4L2_ISP_PARAMS_VERSION_V1)
		, lscGridData_(lscGrid), lscGridSize_(0)
	{
	}

	template <BlockType B>
	auto dcamBlock()
	{
		return dcamCfg_.block<B>();
	}

	template <BlockType B>
	auto ispBlock()
	{
		return ispCfg_.block<B>();
	}

	V4L2Params<dcam_details::param_traits> dcamCfg_;
	V4L2Params<isp_details::param_traits> ispCfg_;

	Span<uint8_t> lscGridData_;
	size_t lscGridSize_;
};

} /* namespace ipa::unisoc */

} /* namespace libcamera */
