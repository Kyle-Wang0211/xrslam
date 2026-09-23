// [pw 2026-09-22 逐帧内参] XRSLAMImage::ext 的消费规则 + 布局静态断言。
//
// 来历:调研判决书 docs/research/adaptive_focus_vio_survey_20260922.md §3.4/§3.7——
// 上游在 XRSLAMImageExtension 预留了 focal_length/focus_distance 却整仓零消费;
// 本次在它尾部追加 fx/fy/cx/cy,并在 XRSLAMImage 原有的 4 字节填充位放 ext_size。
//
// ABI 铁律(出货传输层 PwXrslamTransportCore.cpp 按 40 字节旧 XRSLAMImage 编译):
//   · XRSLAMImage 仍 40 字节,上游六个成员偏移不动(下面 static_assert 钉死)。
//   · 引擎只有在调用者**明确声明** ext_size == sizeof(新 XRSLAMImageExtension) 时
//     才读旧 32 字节之后的内容;ext == nullptr 或 ext_size 是别的值(含 0、32、
//     未初始化的垃圾)一律按旧行为走,不读越界。ext_size 要求**精确相等**而不是
//     >=,是为了把「旧调用者没零初始化、填充位是垃圾、ext 又非空」这种最坏情况
//     的误判概率压到 2^-32;将来若再扩(v3),在 take_frame_intrinsics 里加一个
//     新的 sizeof 分支,不要放宽成 >=。
//   · 单测:pw_tools/regression/test_image_ext_abi.cpp(旧结构放在保护页前,越界即崩)。
#ifndef PW_XRSLAM_IMAGE_EXT_H
#define PW_XRSLAM_IMAGE_EXT_H

#include <vector>   // XRSLAM.h 用了 std::vector 却没自己 include(上游缺陷),先补上
#include <XRSLAM.h>
#include <cmath>
#include <cstddef>
#include <cstdint>

static_assert(sizeof(void *) == 8,
              "XRSLAMImage::ext_size 只在 LP64 下落在原有填充位;ILP32 下它会把 ext 挤后 4 字节, "
              "当前没有任何 32 位出货目标,加 32 位目标前先重审这条 ABI");
static_assert(sizeof(XRSLAMImage) == 40,
              "XRSLAMImage 必须保持 40 字节(出货传输层按 40 字节编译)");
static_assert(offsetof(XRSLAMImage, data) == 0 && offsetof(XRSLAMImage, timeStamp) == 8 &&
                  offsetof(XRSLAMImage, stride) == 16 && offsetof(XRSLAMImage, camera_id) == 20 &&
                  offsetof(XRSLAMImage, channel) == 24 && offsetof(XRSLAMImage, ext) == 32,
              "XRSLAMImage 上游成员偏移被挪动了");
static_assert(offsetof(XRSLAMImage, ext_size) == 28,
              "ext_size 必须正好落在 channel 与 ext 之间原有的 4 字节填充位");
static_assert(sizeof(XRSLAMImage::ext_size) == 4, "ext_size 必须是 4 字节");
static_assert(offsetof(XRSLAMImageExtension, exposure_time) == 0 &&
                  offsetof(XRSLAMImageExtension, default_focus_distance) == 8 &&
                  offsetof(XRSLAMImageExtension, focal_length) == 16 &&
                  offsetof(XRSLAMImageExtension, focus_distance) == 24,
              "XRSLAMImageExtension 上游四个 double 的偏移被挪动了");
static_assert(offsetof(XRSLAMImageExtension, intrinsics_fxfycxcy) == XRSLAM_IMAGE_EXTENSION_LEGACY_SIZE,
              "新字段必须紧接在上游 32 字节之后(只在尾部追加)");
static_assert(sizeof(XRSLAMImageExtension) == 72, "XRSLAMImageExtension v2 应为 72 字节;改布局先改这里和 take_frame_intrinsics");

namespace xrslam::pw {

/// 调用者给了合法的当帧 K ⇒ 填 out_fxfycxcy 并返回 true;否则 false 且不读 ext 的旧 32 字节之外。
inline bool take_frame_intrinsics(const XRSLAMImage *img, double out_fxfycxcy[4]) {
    if (img == nullptr || img->ext == nullptr)
        return false;                                            // 上游三处调用者 / 未挂 ext
    if (img->ext_size != static_cast<unsigned int>(sizeof(XRSLAMImageExtension)))
        return false;                                            // 0(零初始化旧调用者)、32(旧结构)、垃圾:都不读尾部
    const XRSLAMImageExtension *e = img->ext;
    if (e->has_intrinsics != 1)
        return false;
    const double *k = e->intrinsics_fxfycxcy;
    for (int i = 0; i < 4; ++i)
        if (!std::isfinite(k[i]))
            return false;
    if (!(k[0] > 0.0) || !(k[1] > 0.0))
        return false;
    for (int i = 0; i < 4; ++i)
        out_fxfycxcy[i] = k[i];
    return true;
}

} // namespace xrslam::pw

#endif // PW_XRSLAM_IMAGE_EXT_H
