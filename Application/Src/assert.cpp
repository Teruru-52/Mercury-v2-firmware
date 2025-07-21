#include <cstdio>

/**
 * @brief
assert()が呼ばれた際に、ファイル名、行番号、関数名、失敗した条件を表示する

```cpp

  #include <cassert>

  int main() {

    int a = 0;

    \/\/ a==1でなければassertionが発生し、動作が停止する

    assert(a == 1);

  }

 * ```
 *
 * 結果 `assertion "false" failed: file "main.c", line 7, function: int main()`
 *
 */
extern "C" void __wrap___assert_func(const char* file, int line,
                                     const char* func, const char* failedexpr) {
  printf("\n\nassertion \"%s\" failed: file \"%s\", line %d%s%s\n", failedexpr,
         file, line, func ? ", function: " : "", func ? func : "");
  for (;;) {}
}