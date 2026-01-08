#pragma once

#include <functional>

class AllMotors;

class CyclicSession{
 public:
  using Callback = std::function<void(AllMotors&, bool&)>;

  CyclicSession();
  ~CyclicSession();

  void setCallback(Callback cb);
  void run(AllMotors& motors, bool& break_loop);

 private:
  Callback callback_;
};
