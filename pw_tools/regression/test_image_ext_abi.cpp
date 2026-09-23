// test_image_ext_abi.cpp — XRSLAMImage/XRSLAMImageExtension 的 ABI 兼容单测(无需链引擎)。
//
// 证明的事:旧调用者(按上游 32 字节 XRSLAMImageExtension / 40 字节 XRSLAMImage 编译)
// 把旧结构交给新引擎时,引擎既不改行为也不读越界。做法是把 32 字节旧结构放在一页的
// **末尾**,紧接着一页 PROT_NONE 的保护页:引擎只要多读一个字节就 SIGSEGV。
//
//   c++ -std=c++17 -I xrslam-interface/include -I xrslam-interface/src \
//       -o /tmp/test_image_ext_abi pw_tools/regression/test_image_ext_abi.cpp && /tmp/test_image_ext_abi
#include "XRSLAMImageExt.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>

static int failures = 0;
#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) {                                                            \
            printf("  ok   %s\n", msg);                                        \
        } else {                                                               \
            printf("  FAIL %s\n", msg);                                        \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

// 上游 32 字节旧结构,逐字抄 4beb1a9 的 XRSLAM.h:33-38。
struct LegacyExt {
    double exposure_time;
    double default_focus_distance;
    double focal_length;
    double focus_distance;
};
static_assert(sizeof(LegacyExt) == XRSLAM_IMAGE_EXTENSION_LEGACY_SIZE, "legacy struct must be 32 bytes");

int main() {
    printf("static_asserts in XRSLAMImageExt.h passed at compile time "
           "(sizeof XRSLAMImage=%zu, ext offset=%zu, ext_size offset=%zu, sizeof ext v2=%zu)\n",
           sizeof(XRSLAMImage), offsetof(XRSLAMImage, ext), offsetof(XRSLAMImage, ext_size),
           sizeof(XRSLAMImageExtension));

    // ── 保护页:page[0] 可读写,page[1] PROT_NONE;旧结构贴在 page[0] 末尾 ──
    const long pg = sysconf(_SC_PAGESIZE);
    unsigned char *mem = static_cast<unsigned char *>(
        mmap(nullptr, 2 * pg, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0));
    if (mem == MAP_FAILED) { perror("mmap"); return 2; }
    if (mprotect(mem + pg, pg, PROT_NONE) != 0) { perror("mprotect"); return 2; }
    auto *legacy = reinterpret_cast<LegacyExt *>(mem + pg - sizeof(LegacyExt));
    legacy->exposure_time = 0.0083; legacy->default_focus_distance = 0.835;
    legacy->focal_length = 1347.79; legacy->focus_distance = 0.3;   // 旧调用者可能填了这些
    auto *legacy_as_new = reinterpret_cast<XRSLAMImageExtension *>(legacy);   // 引擎看到的类型

    double k[4] = {-1, -1, -1, -1};

    // 1. 出货传输层的路径:XRSLAMImage image{} ⇒ 全零(含 ext_size),ext=nullptr
    {
        XRSLAMImage img{};
        CHECK(img.ext_size == 0 && img.ext == nullptr, "XRSLAMImage{} 零初始化 ⇒ ext_size==0, ext==nullptr(出货传输层路径)");
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "ext==nullptr ⇒ 不消费");
    }
    // 2. 旧调用者:传 32 字节旧结构,ext_size 零初始化
    {
        XRSLAMImage img{};
        img.ext = legacy_as_new; img.ext_size = 0;
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "旧结构 + ext_size=0 ⇒ 不消费、不越界(保护页未触发)");
    }
    // 3. 旧调用者声明了自己的 sizeof(32)
    {
        XRSLAMImage img{};
        img.ext = legacy_as_new; img.ext_size = XRSLAM_IMAGE_EXTENSION_LEGACY_SIZE;
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "旧结构 + ext_size=32 ⇒ 不消费、不越界");
    }
    // 4. 旧调用者没零初始化,填充位是垃圾(取几个典型垃圾值,只要 != 72 都不能读尾部)
    {
        const unsigned int garbage[] = {1u, 31u, 33u, 40u, 64u, 71u, 73u, 0xdeadbeefu, 0xffffffffu};
        bool all_rejected = true;
        for (unsigned int g : garbage) {
            XRSLAMImage img{};
            img.ext = legacy_as_new; img.ext_size = g;
            if (xrslam::pw::take_frame_intrinsics(&img, k)) all_rejected = false;
        }
        CHECK(all_rejected, "旧结构 + 垃圾 ext_size(≠72)⇒ 全部不消费、不越界");
    }
    // 5. 新调用者:完整 v2 结构 + has_intrinsics=1 ⇒ 消费
    {
        XRSLAMImageExtension e{};
        e.intrinsics_fxfycxcy[0] = 1347.79; e.intrinsics_fxfycxcy[1] = 1347.79;
        e.intrinsics_fxfycxcy[2] = 957.47;  e.intrinsics_fxfycxcy[3] = 718.96;
        e.has_intrinsics = 1;
        XRSLAMImage img{};
        img.ext = &e; img.ext_size = sizeof(e);
        const bool ok = xrslam::pw::take_frame_intrinsics(&img, k);
        CHECK(ok && k[0] == 1347.79 && k[1] == 1347.79 && k[2] == 957.47 && k[3] == 718.96,
              "v2 结构 + has_intrinsics=1 ⇒ 消费,fx fy cx cy 逐值一致");
        // 6. 同一结构但标志未置 / 标志为 2 ⇒ 不消费
        e.has_intrinsics = 0;
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "v2 结构 + has_intrinsics=0 ⇒ 不消费");
        e.has_intrinsics = 2;
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "v2 结构 + has_intrinsics=2 ⇒ 不消费(只认 ==1)");
        // 7. 非法数值
        e.has_intrinsics = 1; e.intrinsics_fxfycxcy[0] = NAN;
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "fx=NaN ⇒ 不消费");
        e.intrinsics_fxfycxcy[0] = 0.0;
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "fx=0 ⇒ 不消费");
        e.intrinsics_fxfycxcy[0] = INFINITY;
        CHECK(!xrslam::pw::take_frame_intrinsics(&img, k), "fx=inf ⇒ 不消费");
    }
    // 8. 新结构的前 32 字节与旧结构逐字节同布局:把旧结构 memcpy 进新结构前 32 字节,四个 double 原样可读
    {
        XRSLAMImageExtension e{};
        std::memcpy(&e, legacy, sizeof(LegacyExt));
        CHECK(e.exposure_time == 0.0083 && e.default_focus_distance == 0.835 &&
              e.focal_length == 1347.79 && e.focus_distance == 0.3,
              "v2 前 32 字节 == 上游旧结构(尾部追加,前缀逐字节兼容)");
    }
    munmap(mem, 2 * pg);
    printf(failures ? "FAILED %d\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}
