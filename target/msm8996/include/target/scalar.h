/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef Scalar_H
#define Scalar_H


#define MAX_PLANES 4

enum {
  SCALAR_FLIP_LR = 0x0001,
  SCALAR_FLIP_UD = 0x0002,
  SCALAR_DUAL_PIPE = 0x0004,
};

struct HwRect {
  uint32_t x;
  uint32_t y;
  uint32_t w;
  uint32_t h;
};

struct PixelData {
  /*
   * Number of pixels extension in left, right, top and bottom directions
   * for all color components. This pixel value for each color component
   * should be sum of fetch and repeat pixels.
   */
  int extension[MAX_PLANES];

  /*
   * Number of pixels needs to be overfetched in left, right, top and
   * bottom directions from source image for scaling.
   */
  int overfetch[MAX_PLANES];

  /*
   * Number of pixels needs to be overfetched in left, right, top and
   * bottom directions from source image for scaling.
   */
  int repeat[MAX_PLANES];
};

struct Scale {

  uint8_t enable_pxl_ext;

  /* Scaling Data */
  int init_phase_x[MAX_PLANES];
  int phase_step_x[MAX_PLANES];
  int init_phase_y[MAX_PLANES];
  int phase_step_y[MAX_PLANES];

  struct PixelData left;
  struct PixelData top;
  struct PixelData right;
  struct PixelData bottom;

  uint32_t roi_width[MAX_PLANES];

};

struct PipeInfo {
  int id;
  uint32_t flags;
  struct HwRect src_rect;
  struct HwRect dst_rect;
  struct Scale* scale_data;
  uint8_t horz_deci;
  uint8_t vert_deci;
  uint32_t src_width;
  uint32_t src_height;
};

struct LayerInfo {
  uint32_t src_format;
  struct PipeInfo left_pipe;
  struct PipeInfo right_pipe;
};

struct YuvScaleData {
  uint8_t chroma_sample_v[2];
  uint8_t chroma_sample_h[2];
};


int singleQseedScalar(struct LayerInfo* layer);
int singleRgbScalar(struct LayerInfo* layer);
int dualQseedScalar(struct LayerInfo* layer);
int dualRgbScalar(struct LayerInfo* layer);

#endif
