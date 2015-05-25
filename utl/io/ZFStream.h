#ifndef _utl_io_ZFStream_h_
#define _utl_io_ZFStream_h_

#include <utl/io/ZStream.h>
#include <boost/iostreams/positioning.hpp>
#include <boost/iostreams/optimal_buffer_size.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/iostreams/stream.hpp>
#include <fstream>


namespace utl { namespace io {

  namespace detail {

    inline
    k::ZStream::EZipType
    DeduceZipType(const std::string& fileName, const k::ZStream::EZipType zipType)
    {
      if (zipType & k::ZStream::eAutomatic) {
        const std::size_t len = fileName.length();
        if (fileName.substr(len - 4) == ".bz2")
          return k::ZStream::eBZip2;
        else if (fileName.substr(len - 3) == ".gz")
          return k::ZStream::eGZip;
        else
          return k::ZStream::eNone;
      }
      return zipType;
    }

  }


  class ZFStreamSource {
  private:
    struct Impl {
      Impl() : fNOpenCalls(0) { }

      void
      Init()
      {
        ++fNOpenCalls;
        fStream.close();
        fStream.open(fFileName.c_str(), fOpenMode);
        fPosition = 0;
        fFilter.Push(fStream, fZipType);
      }

      unsigned int fNOpenCalls;
      std::string fFileName;
      k::ZStream::EZipType fZipType;
      std::ios_base::openmode fOpenMode;
      std::ifstream fStream;
      std::size_t fPosition;
      detail::ZIFilter fFilter;
    };

  public:
    typedef char char_type;
    struct category :
      boost::iostreams::input_seekable,
      boost::iostreams::device_tag,
      boost::iostreams::closable_tag
    { };

    ZFStreamSource() { }

    ZFStreamSource(const std::string& fileName, const k::ZStream::EZipType zipType)
    { Open(fileName, zipType); }

    ZFStreamSource(const std::string& fileName,
                   const std::ios_base::openmode openMode = std::ios_base::in,
                   const k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
    { Open(fileName, openMode, zipType); }

    void Open(const std::string& fileName, const k::ZStream::EZipType zipType)
    { Open(fileName, std::ios_base::in, zipType); }

    void
    Open(const std::string& fileName,
         const std::ios_base::openmode openMode = std::ios_base::in,
         const k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
    {
      fImpl.reset(new Impl);
      fImpl->fFileName = fileName;
      fImpl->fZipType = detail::DeduceZipType(fileName, zipType);
      fImpl->fOpenMode =
        (fImpl->fZipType == k::ZStream::eNone ? openMode : std::ios_base::binary);
      fImpl->Init();
    }

    void Close() { fImpl.reset(); }

    std::streamsize
    read(char* const s, const std::streamsize n)
    {
      const std::streamsize size = boost::iostreams::read(fImpl->fFilter, s, n);
      if (size > 0)
        fImpl->fPosition += size;
      return size;
    }

    boost::iostreams::stream_offset
    seek(const boost::iostreams::stream_offset off, const std::ios_base::seekdir way)
    {
      const std::size_t old = fImpl->fPosition;
      // determine new value of fPosition
      std::size_t next;
      if (way == std::ios_base::beg) {
        if (off < 0)
          throw std::ios_base::failure("bad seek offset");
        next = off;
      } else if (way == std::ios_base::cur)
        next = old + off;
      else
        throw std::ios_base::failure("bad seek direction");

      if (next == old)
        return fImpl->fPosition;

      std::size_t skip;

      if (next > old)
        skip = next - old;
      else {
        fImpl->fFilter.reset();
        fImpl->fStream.close();
        fImpl->Init();
        skip = next;
      }

      const std::streamsize chunk = boost::iostreams::optimal_buffer_size(fImpl->fFilter);
      char buff[chunk];

      for (std::size_t i = 0, n = skip/chunk; i < n; ++i)
        if (read(buff, chunk) != chunk)
          return -1; // EOF

      const std::streamsize rest = skip % chunk;
      if (rest && read(buff, rest) != rest)
        return -1; // EOF

      if (fImpl->fPosition != next)
        throw std::ios_base::failure("bad seek result");

      return fImpl->fPosition;
    }

    void close() { Close(); }

    std::size_t tellg() const { return fImpl ? fImpl->fPosition : 0; }

    int GetNOpenCalls() const { return fImpl ? int(fImpl->fNOpenCalls) : -1; }

    bool IsComplete() const { return fImpl && fImpl->fFilter.is_complete(); }

    k::ZStream::EZipType GetZipType() const { return fImpl->fZipType; }

  private:
    boost::shared_ptr<Impl> fImpl;
  };


  /*
    \class ZIFStream ZFStream.h "utl/ZFStream.h"

    NB: the interface is the same as the usual std::istream type
    in order for the class to be used as a drop-in replacement for
    std::ifstream.

    Usage:
    \code
    {
      ZIFStream ifs("foo.bz2");
      string h;
      ifs >> h;
      int i;
      ifs >> i;
    }
    \endcode

    \author Darko Veberic
    \date 12 May 2010
    \date 20 Oct 2011 redesign
    \version $Id: ZFStream.h 3015 2013-08-31 13:19:14Z darko $
  */

  class ZIFStream : public boost::iostreams::stream<ZFStreamSource> {
  private:
    typedef boost::iostreams::stream<ZFStreamSource> Base;

  public:
    ZIFStream() : Base(ZFStreamSource()) { }

    ZIFStream(const std::string& fileName, const k::ZStream::EZipType zipType)
      : Base(ZFStreamSource(fileName, zipType)) { }

    ZIFStream(const std::string& fileName,
              const std::ios_base::openmode openMode = std::ios_base::in,
              const k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
      : Base(ZFStreamSource(fileName, openMode, zipType)) { }

    void Open(const std::string& fileName, const k::ZStream::EZipType zipType)
    { (**this).Open(fileName, std::ios_base::in, zipType); }

    void Open(const std::string& fileName,
              const std::ios_base::openmode openMode = std::ios_base::in,
              const k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
    { (**this).Open(fileName, openMode, zipType); }

    void open(const std::string& fileName,
              const std::ios_base::openmode openMode = std::ios_base::in)
    { Open(fileName, openMode, k::ZStream::eAutomatic); }

    int GetNOpenCalls() { return this->is_open() ? (**this).GetNOpenCalls() : -1; }

    bool IsComplete() { return this->is_open() && (**this).IsComplete(); }

    k::ZStream::EZipType GetZipType() { return (**this).GetZipType(); }
  };


  //


  class ZFStreamSink {
  private:
    struct Impl {
      Impl(const std::string& fileName,
           const std::ios_base::openmode openMode,
           const k::ZStream::EZipType zipType)
        : fStream(fileName.c_str(), openMode), fFilter(fStream, zipType), fZipType(zipType) { }
      std::ofstream fStream;
      detail::ZOFilter fFilter;
      k::ZStream::EZipType fZipType;
    };

  public:
    typedef char char_type;
    struct category :
      boost::iostreams::output,
      boost::iostreams::device_tag,
      boost::iostreams::closable_tag,
      boost::iostreams::flushable_tag
    { };

    ZFStreamSink() { }

    ZFStreamSink(const std::string& fileName, const k::ZStream::EZipType zipType)
    { Open(fileName, zipType); }

    ZFStreamSink(const std::string& fileName,
                 const std::ios_base::openmode openMode = std::ios_base::out,
                 const k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
    { Open(fileName, openMode, zipType); }

    void Open(const std::string& fileName, const k::ZStream::EZipType zipType)
    { Open(fileName, std::ios_base::out, zipType); }

    void
    Open(const std::string& fileName,
         const std::ios_base::openmode openMode = std::ios_base::out,
         k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
    {
      zipType = detail::DeduceZipType(fileName, zipType);
      const std::ios_base::openmode om =
        (zipType == k::ZStream::eNone ? openMode : std::ios_base::binary);
      fImpl.reset(new Impl(fileName, om, zipType));
    }

    void Close() { fImpl.reset(); }

    std::streamsize write(const char* const s, const std::streamsize n)
    { return boost::iostreams::write(fImpl->fFilter, s, n); }

    void close() { Close(); }

    bool flush() { return fImpl->fFilter.flush(); }

    bool IsComplete() const { return fImpl && fImpl->fFilter.is_complete(); }

    k::ZStream::EZipType GetZipType() const { return fImpl->fZipType; }

  private:
    boost::shared_ptr<Impl> fImpl;
  };


  /*
    \class ZOFStream ZFStream.h "utl/ZFStream.h"

    NB: the interface is the same as the usual std::ostream type
    in order for the class to be used as a drop-in replacement for
    std::ofstream.

    Usage:
    \code
    {
      OZFStream ofs("foo.bz2");
      ofs << "hallo " << 13 << endl;
    }
    \endcode

    \author Darko Veberic
    \date 12 May 2010
    \date 20 Oct 2011 redesign
    \version $Id: ZFStream.h 3015 2013-08-31 13:19:14Z darko $
  */

  class ZOFStream : public boost::iostreams::stream<ZFStreamSink> {
  private:
    typedef boost::iostreams::stream<ZFStreamSink> Base;

  public:
    ZOFStream() : Base(ZFStreamSink()) { }

    ZOFStream(const std::string& fileName, const k::ZStream::EZipType zipType)
      : Base(ZFStreamSink(fileName, zipType)) { }

    ZOFStream(const std::string& fileName,
              const std::ios_base::openmode openMode = std::ios_base::out,
              const k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
      : Base(ZFStreamSink(fileName, openMode, zipType)) { }

    void Open(const std::string& fileName, const k::ZStream::EZipType zipType)
    { (**this).Open(fileName, std::ios_base::out, zipType); }

    void Open(const std::string& fileName,
              const std::ios_base::openmode openMode = std::ios_base::out,
              const k::ZStream::EZipType zipType = k::ZStream::eAutomatic)
    { (**this).Open(fileName, openMode, zipType); }

    void open(const std::string& fileName,
              const std::ios_base::openmode openMode = std::ios_base::out)
    { Open(fileName, openMode, k::ZStream::eAutomatic); }

    bool IsComplete() { return this->is_open() && (**this).IsComplete(); }

    k::ZStream::EZipType GetZipType() { return (**this).GetZipType(); }
  };

  
} }


#endif
