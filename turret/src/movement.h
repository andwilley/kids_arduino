#ifndef TURRET_MOVEMENT_H_
#define TURRET_MOVEMENT_H_

namespace turret {

// TODO
class Movement {
public:
  Movement(bool is_tracking = true) : is_tracking_(is_tracking) {}

private:
  bool is_tracking_;
};

} // namespace turret

#endif // TURRET_MOVEMENT_H_
