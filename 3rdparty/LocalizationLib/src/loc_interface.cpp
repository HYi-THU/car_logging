#include "loc_interface.h"
#include "localization_impl.h"

std::shared_ptr<LocInterface>
LocInterface::make() {
  std::shared_ptr<LocInterface> loc = std::make_shared<Localization>();
  return loc;
}