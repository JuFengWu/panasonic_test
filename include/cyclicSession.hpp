#pragma once

#include <functional>

class Motors;

class CyclicSession{
 public:
  using Callback = std::function<void(Motors&, bool&)>;

  CyclicSession();
  ~CyclicSession();

  void setCallback(Callback cb);

 private:
  Callback callback_;
};
