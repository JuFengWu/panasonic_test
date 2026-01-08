#include "cyclicSession.hpp"

CyclicSession::CyclicSession() = default;

CyclicSession::~CyclicSession() = default;

void CyclicSession::setCallback(Callback cb) { callback_ = std::move(cb); }

void CyclicSession::run(AllMotors& motors, bool& break_loop)
{
  if (callback_) {
    callback_(motors, break_loop);
  }
}
