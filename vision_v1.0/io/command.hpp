#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  float yaw;
  float pitch;
};

}  // namespace io

#endif  // IO__COMMAND_HPP