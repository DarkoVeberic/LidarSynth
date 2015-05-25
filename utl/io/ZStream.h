#ifndef _utl_io_ZStream_h_
#define _utl_io_ZStream_h_

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/shared_ptr.hpp>

/*
  \author Darko Veberic
  \version $Id: ZStream.h 1975 2011-11-06 12:39:54Z darko $
*/


namespace utl { namespace io {

  namespace k { namespace ZStream {

    enum EZipType {
      eNone = 0,
      eGZip = 1,
      eBZip2 = 2,
      eXZip = 3,
      //ePipe = 8,
      eAutomatic = 16,
    };

  } }


  namespace detail {

    template<
      class FilteringStream,
      class GZipCompressor,
      class BZip2Compressor
    >
    struct ZFilter : public FilteringStream {
      ZFilter() : FilteringStream() { }

      template<class Stream>
      ZFilter(Stream& stream, const k::ZStream::EZipType zipType)
      { Push(stream, zipType); }

      ~ZFilter() { this->reset(); }

      template<class Stream>
      void
      Push(Stream& stream, const k::ZStream::EZipType zipType)
      {
        this->reset();
        if (zipType >= 4)
          throw std::invalid_argument("only zip options allowed for ZFilter");
        switch (zipType & 3) {
        case k::ZStream::eNone:
          break;
        case k::ZStream::eGZip:
          this->push(GZipCompressor());
          break;
        case k::ZStream::eBZip2:
          this->push(BZip2Compressor());
          break;
        case k::ZStream::eXZip:
          throw std::invalid_argument("eXZip not implemented yet");
        }
        this->push(stream);
        //this->set_auto_close(true);
      }
    };

    typedef ZFilter<
      boost::iostreams::filtering_istream,
      boost::iostreams::gzip_decompressor,
      boost::iostreams::bzip2_decompressor
    > ZIFilter;

    typedef ZFilter<
      boost::iostreams::filtering_ostream,
      boost::iostreams::gzip_compressor,
      boost::iostreams::bzip2_compressor
    > ZOFilter;

  }


  //


  class ZStreamSource {
  public:
    typedef char char_type;
    struct category :
      boost::iostreams::input,
      boost::iostreams::device_tag,
      boost::iostreams::closable_tag
    { };

    ZStreamSource() { }

    template<class IStream>
    ZStreamSource(IStream& istream, const k::ZStream::EZipType zipType = k::ZStream::eNone)
    { Attach(istream, zipType); }

    template<class IStream>
    void Attach(IStream& istream, const k::ZStream::EZipType zipType = k::ZStream::eNone)
    { fImpl.reset(new Impl(istream, zipType)); }
    
    void Close() { fImpl->fFilter.reset(); }

    std::streamsize read(char* const s, const std::streamsize n)
    { return boost::iostreams::read(fImpl->fFilter, s, n); }

    void close() { Close(); }

    bool IsComplete() const { return fImpl && fImpl->fFilter.is_complete(); }

  private:
    struct Impl {
      template<class Stream>
      Impl(Stream& stream, const k::ZStream::EZipType zipType) { fFilter.Push(stream, zipType); }
      detail::ZIFilter fFilter;
    };
    boost::shared_ptr<Impl> fImpl;
  };

  typedef boost::iostreams::stream<ZStreamSource> ZIStream;


  //


  class ZStreamSink {
  public:
    typedef char char_type;
    struct category :
      boost::iostreams::output,
      boost::iostreams::device_tag,
      boost::iostreams::closable_tag,
      boost::iostreams::flushable_tag
    { };

    ZStreamSink() { }

    template<class OStream>
    ZStreamSink(OStream& ostream, const k::ZStream::EZipType zipType = k::ZStream::eNone)
    { Attach(ostream, zipType); }

    template<class OStream>
    void Attach(OStream& ostream, const k::ZStream::EZipType zipType = k::ZStream::eNone)
    { fImpl.reset(new Impl(ostream, zipType)); }

    void Close() { fImpl.reset(); }

    std::streamsize write(const char* const s, const std::streamsize n)
    { return boost::iostreams::write(fImpl->fFilter, s, n); }

    void close() { Close(); }

    bool flush() { return fImpl->fFilter.flush(); }

    bool IsComplete() const { return fImpl->fFilter.is_complete(); }

  private:
    struct Impl {
      template<class Stream>
      Impl(Stream& stream, const k::ZStream::EZipType zipType) { fFilter.Push(stream, zipType); }
      detail::ZOFilter fFilter;
    };
    boost::shared_ptr<Impl> fImpl;
  };

  typedef boost::iostreams::stream<ZStreamSink> ZOStream;

} }


#endif
