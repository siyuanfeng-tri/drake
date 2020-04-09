#include "drake/systems/sensors/mask_converter.h"

namespace drake {
namespace systems {
namespace sensors {

MaskConverter::MaskConverter(int body_id, uint8_t val, int width, int height)
    : body_id_(body_id), val_(val) {
  ImageGrey8U binary_image(width, height);
  this->DeclareAbstractOutputPort("binary_image", binary_image,
                                  &MaskConverter::ConvertLabelImage);
  ImageLabel16I label_image(width, height);
  this->DeclareAbstractInputPort("label_image",
                                 Value<ImageLabel16I>(label_image));
}

void MaskConverter::ConvertLabelImage(const Context<double>& context,
                                      ImageGrey8U* binary_image) const {
  const auto& label_image = get_input_port(0).Eval<ImageLabel16I>(context);
  for (int x = 0; x < label_image.width(); x++) {
    for (int y = 0; y < label_image.height(); y++) {
      if (*label_image.at(x, y) == body_id_) {
        *binary_image->at(x, y) = static_cast<uint8_t>(val_);
      } else {
        *binary_image->at(x, y) = 0;
      }
    }
  }
}

PotatoMaskConverter::PotatoMaskConverter(uint8_t upper_thresh, int width,
                                         int height)
    : thresh_val_(upper_thresh) {
  ImageGrey8U binary_image(width, height);
  this->DeclareAbstractOutputPort("binary_image", binary_image,
                                  &PotatoMaskConverter::ConvertLabelImage);
  ImageLabel16I label_image(width, height);
  this->DeclareAbstractInputPort("label_image",
                                 Value<ImageLabel16I>(label_image));
}

void PotatoMaskConverter::ConvertLabelImage(const Context<double>& context,
                                            ImageGrey8U* binary_image) const {
  const auto& label_image = get_input_port(0).Eval<ImageLabel16I>(context);
  for (int x = 0; x < label_image.width(); x++) {
    for (int y = 0; y < label_image.height(); y++) {
      uint8_t input = static_cast<uint8_t>(*label_image.at(x, y));
      if (input < thresh_val_) {
        *binary_image->at(x, y) = input;
      } else {
        *binary_image->at(x, y) = 0;
      }
    }
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
