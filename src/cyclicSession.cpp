#include "cyclicSession.hpp"

CyclicSession::CyclicSession() = default;

CyclicSession::~CyclicSession() = default;

void CyclicSession::setCallback(Callback cb) { callback_ = std::move(cb); }

void CyclicSession::run(AllMotors& motors, bool& cycle_shutdown_request)
{
  if (callback_) {
    callback_(motors, cycle_shutdown_request);
  }
}
