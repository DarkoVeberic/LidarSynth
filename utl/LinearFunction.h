#include <iostream>


namespace utl {

  class LinearFunction {
  public:
    LinearFunction(const double slope = 0, const double offset = 0)
      : fSlope(slope), fOffset(offset) { }

    double operator()(const double x) const
    { return fSlope*x + fOffset; }

    double Inverse(const double y) const
    { return (y - fOffset) / fSlope; }

    double GetSlope() const { return fSlope; }

    double GetOffset() const { return fOffset; }

  private:
    double fSlope;
    double fOffset;
  };


  inline
  std::ostream&
  operator<<(std::ostream& os, const LinearFunction& f)
  {
    return os << f.GetSlope() << "*x + " << f.GetOffset();
  }

}
