#include <cmath>
#include <utl/Math.h>

using namespace std;


namespace utl {

  double
  LogFactorial(unsigned int n)
  {
    if (n < 2)
      return 0;
    if (n <= 12) {
      unsigned int f = 2;
      for (unsigned int i = 3; i <= n; ++i)
        f *= i;
      return log(double(f));
    }
    // Stirling formula for log(n!) = log(Gamma(n+1)) for large n 
    ++n;
    return 0.5*log(2*M_PI/n) +
      n*(log(n + 1/(12.*n - 1/(10.*n))) - 1);
  }

}
