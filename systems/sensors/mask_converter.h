#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

class MaskConverter final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaskConverter)

  /**
   * This is only useful when wired up to the default label rendering, which
   * outputs body id.
   *
   * Takes in a 16bit label image, and outputs a 8bit grey scale image. For
   * every pixel in the input image whose label equals `body_id`, the output
   * image's corresponding pixel will be `val`, and every thing else will be 0.
   */
  MaskConverter(int body_id, uint8_t val, int width, int height);

 private:
  void ConvertLabelImage(const Context<double>& context,
                         ImageGrey8U* label_image) const;
  int body_id_{};
  uint8_t val_{};
};

class PotatoMaskConverter final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PotatoMaskConverter)

  /**
   * This is only useful for generating masks for potato images, and requires
   * the special label rendering that pays attention to a masked texture.
   *
   * Takes in a 16bit label image, and outputs a 8bit grey scale image.
   * Values in the input image whose value is above `upper_thresh` will be set
   * to zero, and the rest will be perserved.
   */
  PotatoMaskConverter(uint8_t upper_mask_thresh, int width, int height);

 private:
  void ConvertLabelImage(const Context<double>& context,
                         ImageGrey8U* label_image) const;
  uint8_t thresh_val_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
