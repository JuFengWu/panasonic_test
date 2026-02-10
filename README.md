# MotorCode

## Beginner Start
新手請先看 `demo/beginner_demo.cpp`。

## 程式流程（beginner_demo.cpp）
1. **基本設定**  
   設定網路介面名稱、馬達數量、控制週期與模式。
2. **建立並連線**  
   建立 `MotionSystem`，呼叫 `start_connect(...)` 完成連線與初始化。
3. **啟動背景循環**  
   呼叫 `run_async()` 啟動通訊與控制循環（背景 thread 持續交換資料）。
4. **設定 callback（讀取狀態）**  
   透過 `setCallback(...)` 週期性讀取目前位置（不做控制）。
5. **Servo ON**  
   呼叫 `servo_on()` 讓馬達進入可運行狀態。
6. **設定目標值**  
   在 thread 外直接設定目標位置（示範簡單控制指令）。
7. **等待一段時間**  
   讓馬達實際運行一段時間。
8. **Servo OFF 與關閉**  
   呼叫 `servo_off()`，最後 `close()` 釋放資源並結束。

## Cppcheck
### Run Static Analysis
```bash
cmake -S .. -B build
cmake --build build --target cppcheck
```
