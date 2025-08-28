#pragma once
namespace wl
{
enum class Status : int
{
  kHalt = -1,
  kReady = 0,
  kRunning = 1,
  kPreparing = 2
};

constexpr const char* StatusToString(Status status)
{
  switch (status)
  {
    case Status::kHalt:
      return "Halt";
    case Status::kReady:
      return "Ready";
    case Status::kRunning:
      return "Running";
    case Status::kPreparing:
      return "Preparing";
    default:
      return "Unknown";
  }
}
}  // namespace wl