#include <iio.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr uint64_t kRate = 15360000;
constexpr uint64_t kDefaultBandwidth = 10000000;
constexpr size_t kNfft = 1024, kCarriers = 601, kDataBits = 7200;
constexpr size_t kK = 1408, kKpi = 1440, kRows = 45, kCols = 32;
constexpr size_t kDummy = kKpi - (kK + 4);
constexpr uint32_t kGoldSeed = 0x12345678, kGoldNc = 1600;
constexpr double kPi = 3.14159265358979323846;
constexpr unsigned kPayloadCrcOffset = 89;
constexpr unsigned kSerialSize = 16;
constexpr uint16_t kCrcInitial = 0x3692, kCrcPoly = 0x8408;
constexpr double kCoordinateScale = 174533.0, kFeetPerMetre = 3.281;
volatile std::sig_atomic_t g_stop = 0;
void Stop(int) { g_stop = 1; }

struct Options {
    std::vector<uint64_t> hops;
    std::string aircraft_id = "TEST-DRONEID-01";
    std::string output_path;
    double latitude = 51.5007, longitude = -0.1246;
    double altitude_m = 35.0, height_m = 30.0;
    double gain_db = 0.0, peak = 0.7, pad_us = 100.0;
    uint64_t offset_hz = 500000;
    std::string side = "upper";
    unsigned tx_channel = 0;
    uint64_t bandwidth_hz = kDefaultBandwidth;
    uint32_t period_ms = 1000, sequence = 1, count = 0;
};

void Usage(const char *name) {
    std::cerr << "Usage: " << name << " --hops HZ[,HZ...] --gain DB [options]\n"
              << "  --period-ms N       packet start interval (default 1000)\n"
              << "  --bandwidth HZ      TX RF bandwidth (default 10000000)\n"
              << "  --lat DEG --lon DEG --alt M --height M\n"
              << "  --aircraft-id TEXT  up to 16 bytes\n"
              << "  --sequence N --count N (0 means until Ctrl-C)\n"
              << "  --side upper|lower --offset-hz HZ (default upper, 500000)\n"
              << "  --tx-channel 0|1   select TX chain (default 0)\n"
              << "  --peak 0..1 --pad-us N --output FILE (save interleaved CS16 IQ)\n"
              << "Pluto TX is local IIO and uses a 15.36 MS/s waveform.\n";
}

bool Unsigned(const std::string &s, uint64_t &v) {
    if (s.empty() || s[0] == '-') return false;
    char *end = nullptr; errno = 0;
    unsigned long long n = std::strtoull(s.c_str(), &end, 10);
    if (errno || !end || *end) return false;
    v = n; return true;
}
bool Real(const std::string &s, double &v) {
    char *end = nullptr; errno = 0; v = std::strtod(s.c_str(), &end);
    return !errno && end && *end == '\0' && std::isfinite(v);
}
bool ParseHops(const std::string &s, std::vector<uint64_t> &hops) {
    std::stringstream in(s); std::string token;
    while (std::getline(in, token, ',')) {
        uint64_t f = 0;
        if (!Unsigned(token, f) || f < 2400000000ULL || f > 2500000000ULL) return false;
        hops.push_back(f);
    }
    return !hops.empty();
}
bool Parse(int argc, char **argv, Options &o) {
    bool gain_set = false;
    for (int i = 1; i < argc; ++i) {
        std::string key(argv[i]);
        if (key == "--help" || key == "-h") { Usage(argv[0]); std::exit(0); }
        if (i + 1 >= argc) return false;
        std::string v(argv[++i]); uint64_t n = 0;
        if (key == "--hops") { if (!ParseHops(v, o.hops)) return false; }
        else if (key == "--aircraft-id") o.aircraft_id = v;
        else if (key == "--output") o.output_path = v;
        else if (key == "--side") o.side = v;
        else if (key == "--tx-channel") {
            if (!Unsigned(v, n) || n > 1) return false;
            o.tx_channel = static_cast<unsigned>(n);
        }
        else if (key == "--gain") { if (!Real(v, o.gain_db)) return false; gain_set = true; }
        else if (key == "--lat") { if (!Real(v, o.latitude)) return false; }
        else if (key == "--lon") { if (!Real(v, o.longitude)) return false; }
        else if (key == "--alt") { if (!Real(v, o.altitude_m)) return false; }
        else if (key == "--height") { if (!Real(v, o.height_m)) return false; }
        else if (key == "--peak") { if (!Real(v, o.peak)) return false; }
        else if (key == "--pad-us") { if (!Real(v, o.pad_us)) return false; }
        else if (key == "--sequence" || key == "--count" ||
                 key == "--period-ms" || key == "--bandwidth" || key == "--offset-hz") {
            if (!Unsigned(v, n) || n > UINT32_MAX) return false;
            if (key == "--sequence") { if (n > UINT16_MAX) return false; o.sequence = n; }
            else if (key == "--count") o.count = n;
            else if (key == "--period-ms") { if (!n || n > 3600000) return false; o.period_ms = n; }
            else if (key == "--bandwidth") o.bandwidth_hz = n;
            else o.offset_hz = n;
        } else return false;
    }
    return gain_set && !o.hops.empty() && o.aircraft_id.size() <= 16 &&
        (o.side == "upper" || o.side == "lower") && o.offset_hz <= 2000000 &&
        o.offset_hz + 5000000 < kRate/2 &&
        o.gain_db >= -89.75 && o.gain_db <= 0 && o.bandwidth_hz >= 1000000 &&
        o.bandwidth_hz <= 20000000 && o.latitude >= -90 && o.latitude <= 90 &&
        o.longitude >= -180 && o.longitude <= 180 && o.altitude_m >= -10000 &&
        o.altitude_m <= 10000 && o.height_m >= -10000 && o.height_m <= 10000 &&
        o.peak > 0 && o.peak <= 1 && o.pad_us >= 0 && o.pad_us <= 1000000;
}

using Payload = std::array<uint8_t, kPayloadCrcOffset>;
void Put16(Payload &p, size_t at, uint16_t v) { p[at] = v; p[at+1] = v >> 8; }
void Put32(Payload &p, size_t at, int32_t v) {
    uint32_t u = static_cast<uint32_t>(v);
    for (unsigned i=0; i<4; ++i) p[at+i] = static_cast<uint8_t>(u >> (8*i));
}
int16_t ToRaw(double m) {
    long v = std::lround(m * kFeetPerMetre);
    if (v < INT16_MIN || v > INT16_MAX) throw std::runtime_error("altitude/height out of range");
    return static_cast<int16_t>(v);
}
uint16_t Crc16(const uint8_t *p, size_t n) {
    uint16_t crc = kCrcInitial;
    for (size_t i=0; i<n; ++i) {
        crc ^= p[i];
        for (unsigned b=0; b<8; ++b) crc = (crc >> 1) ^ ((crc & 1) ? kCrcPoly : 0);
    }
    return crc;
}
Payload MakePayload(const Options &o) {
    Payload p{}; p[0]=0x5b; p[2]=2;
    Put16(p, 3, static_cast<uint16_t>(o.sequence)); Put16(p, 5, 0x1f13);
    std::copy(o.aircraft_id.begin(), o.aircraft_id.end(), p.begin()+7);
    const double scale = kCoordinateScale;
    Put32(p,23,static_cast<int32_t>(std::llround(o.longitude*scale)));
    Put32(p,27,static_cast<int32_t>(std::llround(o.latitude*scale)));
    Put16(p,31,static_cast<uint16_t>(ToRaw(o.altitude_m)));
    Put16(p,33,static_cast<uint16_t>(ToRaw(o.height_m)));
    Put32(p,51,static_cast<int32_t>(std::llround(o.latitude*scale)));
    Put32(p,55,static_cast<int32_t>(std::llround(o.longitude*scale)));
    Put32(p,59,static_cast<int32_t>(std::llround(o.longitude*scale)));
    Put32(p,63,static_cast<int32_t>(std::llround(o.latitude*scale)));
    p[67]=1; p[68]=kSerialSize;
    std::copy(o.aircraft_id.begin(), o.aircraft_id.end(), p.begin()+69);
    return p;
}

std::vector<uint8_t> Gold(size_t length) {
    std::vector<uint8_t> x1(kGoldNc+length+31), x2(x1.size()); x1[0]=1;
    for (unsigned n=0;n<32;++n) x2[n]=(kGoldSeed>>n)&1;
    for (size_t n=0;n<kGoldNc+length;++n) {
        x1[n+31]=(x1[n+3]^x1[n])&1;
        x2[n+31]=(x2[n+3]^x2[n+2]^x2[n+1]^x2[n])&1;
    }
    std::vector<uint8_t> out(length);
    for (size_t i=0;i<length;++i) out[i]=x1[kGoldNc+i]^x2[kGoldNc+i];
    return out;
}
struct Streams { std::vector<uint8_t> sys, p1, p2; };
void Rsc(const std::vector<uint8_t> &bits, std::vector<uint8_t> &sys,
         std::vector<uint8_t> &parity) {
    sys.resize(bits.size()+3); parity.resize(bits.size()+3);
    uint8_t r1=0,r2=0,r3=0;
    for (size_t i=0;i<bits.size();++i) {
        const uint8_t old1=r1, old2=r2;
        uint8_t a=bits[i]^r2^r3;
        sys[i]=bits[i]; parity[i]=a^r1^r3;
        r1=a; r2=old1; r3=old2;
    }
    for (size_t i=0;i<3;++i) {
        const uint8_t old1=r1, old2=r2;
        uint8_t u=r2^r3, a=u^r2^r3;
        sys[bits.size()+i]=u; parity[bits.size()+i]=a^r1^r3;
        r1=a; r2=old1; r3=old2;
    }
}
Streams Turbo(const std::vector<uint8_t> &info) {
    std::vector<uint8_t> inter(kK);
    for (size_t i=0;i<kK;++i) {
        size_t q=(43*i+88*i*i)%kK; inter[i]=info[q];
    }
    Streams a,b; Rsc(info,a.sys,a.p1); Rsc(inter,b.sys,b.p1);
    Streams out; out.sys.resize(kK+4); out.p1.resize(kK+4); out.p2.resize(kK+4);
    for(size_t i=0;i<kK;++i){out.sys[i]=a.sys[i];out.p1[i]=a.p1[i];out.p2[i]=b.p1[i];}
    out.sys[kK]=a.sys[kK]; out.p2[kK]=a.sys[kK+1]; out.p1[kK+1]=a.sys[kK+2];
    out.p1[kK]=a.p1[kK]; out.sys[kK+1]=a.p1[kK+1]; out.p2[kK+1]=a.p1[kK+2];
    out.sys[kK+2]=b.sys[kK]; out.p2[kK+2]=b.sys[kK+1]; out.p1[kK+3]=b.sys[kK+2];
    out.p1[kK+2]=b.p1[kK]; out.sys[kK+3]=b.p1[kK+1]; out.p2[kK+3]=b.p1[kK+2];
    return out;
}
size_t Rm0(size_t k) {
    static const unsigned perm[32]={0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30,1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31};
    return (k%kRows)*kCols + perm[k/kRows];
}
size_t Rm2(size_t k) {
    static const unsigned perm[32]={0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30,1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31};
    return (perm[k/kRows]+kCols*(k%kRows)+1)%kKpi;
}
std::vector<uint8_t> RateMatch(const Streams &s) {
    struct Ref { int stream; size_t index; bool valid; };
    std::vector<Ref> circular; circular.reserve(3*kKpi);
    for(size_t w=0;w<kKpi;++w){size_t x=Rm0(w);circular.push_back({0,x>=kDummy?x-kDummy:0,x>=kDummy});}
    for(size_t m=0;m<2*kKpi;++m){size_t x;
        if(!(m&1)){x=Rm0(m/2);circular.push_back({1,x>=kDummy?x-kDummy:0,x>=kDummy});}
        else{x=Rm2((m-1)/2);circular.push_back({2,x>=kDummy?x-kDummy:0,x>=kDummy});}}
    std::vector<Ref> valid; for(const Ref &r:circular)if(r.valid)valid.push_back(r);
    size_t first=0;while(first<valid.size()&&valid[first].stream!=0)++first;
    size_t start=(first+valid.size()-4148%valid.size())%valid.size();
    const std::vector<uint8_t> *ss[3]={&s.sys,&s.p1,&s.p2};std::vector<uint8_t> out(kDataBits);
    for(size_t j=0;j<kDataBits;++j){const Ref &r=valid[(start+j)%valid.size()];out[j]=(*ss[r.stream])[r.index];}
    return out;
}
std::complex<float> Qpsk(uint8_t b0,uint8_t b1) {
    const unsigned v=b0|(b1<<1); static const unsigned map[4]={3,2,0,1};
    const unsigned q=map[v]; const float re=(q<2)?1.0f:-1.0f, im=(q==0||q==3)?1.0f:-1.0f;
    return {re*0.7071067811865475f,im*0.7071067811865475f};
}
void Ifft(std::array<std::complex<float>,kNfft> &a) {
    for(size_t i=1,j=0;i<kNfft;++i){size_t bit=kNfft>>1;for(;j&bit;bit>>=1)j^=bit;j^=bit;if(i<j)std::swap(a[i],a[j]);}
    for(size_t len=2;len<=kNfft;len<<=1){const float angle=static_cast<float>(2*kPi/len);
        std::complex<float> root(std::cos(angle),std::sin(angle));
        for(size_t i=0;i<kNfft;i+=len){std::complex<float>w(1,0);for(size_t j=0;j<len/2;++j){auto u=a[i+j],v=a[i+j+len/2]*w;a[i+j]=u+v;a[i+j+len/2]=u-v;w*=root;}}}
    for(auto &v:a)v/=static_cast<float>(kNfft);
}
std::vector<std::complex<float>> Encode(const Payload &body,double peak,double pad_us) {
    std::vector<uint8_t> payload(body.begin(),body.end());uint16_t crc=Crc16(payload.data(),payload.size());
    payload.push_back(static_cast<uint8_t>(crc));payload.push_back(static_cast<uint8_t>(crc>>8));
    std::vector<uint8_t> bits(176*8,0);
    for(size_t i=0;i<payload.size();++i)for(unsigned b=0;b<8;++b)bits[8*i+b]=(payload[i]>>(7-b))&1;
    auto coded=RateMatch(Turbo(bits));auto gold=Gold(kDataBits);
    for(size_t i=0;i<coded.size();++i)coded[i]^=gold[i];
    auto pilotbits=Gold(1200);std::vector<std::complex<float>> pilot(600);
    for(size_t i=0;i<600;++i)pilot[i]=Qpsk(pilotbits[2*i],pilotbits[2*i+1]);
    std::vector<std::complex<float>> out; out.reserve(9880);
    const unsigned cp[9]={80,72,72,72,72,72,72,72,80};
    for(unsigned sym=0;sym<9;++sym){std::array<std::complex<float>,kNfft> f{};
        auto put=[&](size_t carrier,std::complex<float> x){if(carrier<300)f[kNfft-300+carrier]=x;else if(carrier>300)f[carrier-300]=x;};
        if(sym==0){for(size_t i=0;i<600;++i)put(i<300?i:i+1,pilot[i]);}
        else if(sym==3||sym==5){const unsigned root=sym==3?600:147;for(size_t n=0;n<kCarriers;++n){if(n==300)continue;double phase=-kPi*root*n*(n+1)/kCarriers;put(n,{static_cast<float>(std::cos(phase)),static_cast<float>(std::sin(phase))});}}
        else {const size_t slot=(sym==1?0:sym==2?1:sym==4?2:sym==6?3:sym==7?4:5);for(size_t i=0;i<600;++i)put(i<300?i:i+1,Qpsk(coded[slot*1200+2*i],coded[slot*1200+2*i+1]));}
        Ifft(f);for(size_t i=kNfft-cp[sym];i<kNfft;++i)out.push_back(f[i]);out.insert(out.end(),f.begin(),f.end());
    }
    float maxmag=0;for(const auto &x:out)maxmag=std::max(maxmag,std::abs(x));
    if(maxmag<=0)throw std::runtime_error("encoder generated an empty frame");
    const float factor=static_cast<float>(peak/maxmag);for(auto &x:out)x*=factor;
    const size_t padding=static_cast<size_t>(std::llround(kRate*pad_us/1e6));
    out.insert(out.begin(),padding,std::complex<float>(0,0));out.insert(out.end(),padding,std::complex<float>(0,0));
    return out;
}

bool ReadAttr(struct iio_channel *ch,const char *attr,long long &v) {
    return iio_channel_attr_read_longlong(ch,attr,&v)>=0;
}
int WriteAttr(struct iio_channel *ch,const char *attr,long long v) {
    int r=iio_channel_attr_write_longlong(ch,attr,v);if(r<0){char e[128];iio_strerror(-r,e,sizeof(e));std::cerr<<"IIO write "<<attr<<": "<<e<<"\n";}return r;
}
int WriteAttr(struct iio_channel *ch,const char *attr,const char *v) {
    int r=iio_channel_attr_write(ch,attr,v);if(r<0){char e[128];iio_strerror(-r,e,sizeof(e));std::cerr<<"IIO write "<<attr<<": "<<e<<"\n";}return r;
}
void SaveIq(const std::string &path,const std::vector<int16_t>&iq) {
    std::ofstream f(path.c_str(),std::ios::binary|std::ios::trunc);if(!f)throw std::runtime_error("cannot create output IQ file");
    f.write(reinterpret_cast<const char*>(iq.data()),static_cast<std::streamsize>(iq.size()*sizeof(int16_t)));if(!f)throw std::runtime_error("failed writing IQ file");
}
} // namespace

int main(int argc,char **argv) {
    Options o;if(!Parse(argc,argv,o)){Usage(argv[0]);return 2;}
    std::signal(SIGINT,Stop);std::signal(SIGTERM,Stop);
    struct iio_context *ctx=nullptr;struct iio_buffer *buffer=nullptr;
    struct iio_channel *i_chan=nullptr,*q_chan=nullptr;
    bool channels_enabled=false;
    try {
        Payload payload=MakePayload(o);std::vector<std::complex<float>> wave=Encode(payload,o.peak,o.pad_us);
        const double signed_offset=(o.side=="upper"?1.0:-1.0)*static_cast<double>(o.offset_hz);
        const double phase_step=2.0*kPi*signed_offset/kRate;
        const std::complex<double> phase_inc(std::cos(phase_step),std::sin(phase_step));
        std::complex<double> phase(1.0,0.0);
        for(auto &sample:wave){sample*=std::complex<float>(static_cast<float>(phase.real()),static_cast<float>(phase.imag()));phase*=phase_inc;}
        std::vector<int16_t> iq(wave.size()*2);
        for(size_t n=0;n<wave.size();++n){iq[2*n]=static_cast<int16_t>(std::lrint(std::max(-1.0f,std::min(1.0f,wave[n].real()))*0.70f*32767));iq[2*n+1]=static_cast<int16_t>(std::lrint(std::max(-1.0f,std::min(1.0f,wave[n].imag()))*0.70f*32767));}
        if(!o.output_path.empty())SaveIq(o.output_path,iq);
        ctx=iio_create_default_context();if(!ctx)throw std::runtime_error("cannot create local IIO context");
        auto *phy=iio_context_find_device(ctx,"ad9361-phy");auto *tx=iio_context_find_device(ctx,"cf-ad9361-dds-core-lpc");
        if(!phy||!tx)throw std::runtime_error("local Pluto AD9361/TX DMA IIO devices not found");
        auto *lo=iio_device_find_channel(phy,"altvoltage1",true);
        const char *phy_tx_name=o.tx_channel==0?"voltage0":"voltage1";
        const char *i_name=o.tx_channel==0?"voltage0":"voltage2";
        const char *q_name=o.tx_channel==0?"voltage1":"voltage3";
        auto *phy_tx=iio_device_find_channel(phy,phy_tx_name,true);
        i_chan=iio_device_find_channel(tx,i_name,true);q_chan=iio_device_find_channel(tx,q_name,true);
        auto *phy_rx=iio_device_find_channel(phy,"voltage0",false);
        if(!lo||!phy_tx||!phy_rx)throw std::runtime_error("required Pluto AD9361 channels missing");
        if(!i_chan||!q_chan) {
            if(o.tx_channel==1)
                throw std::runtime_error("TX chain 1 is not exposed by the loaded FPGA/device-tree IIO layout (expected voltage2 and voltage3)");
            throw std::runtime_error("selected TX I/Q DMA channels are missing");
        }

        long long current_rate=0;
        if(!ReadAttr(phy_tx,"sampling_frequency",current_rate))throw std::runtime_error("cannot read current TX sample rate");
        long long rx_rate=0;const bool have_rx_rate=ReadAttr(phy_rx,"sampling_frequency",rx_rate);
        if(!have_rx_rate || rx_rate!=static_cast<long long>(kRate))
            throw std::runtime_error("RX must already be at 15.36 MS/s; refusing to transmit with a mismatched or unreadable RX rate");
        if(current_rate!=static_cast<long long>(kRate)) {
            if(WriteAttr(phy_tx,"sampling_frequency",static_cast<long long>(kRate))<0)
                throw std::runtime_error("cannot set TX sample rate to 15.36 MS/s");
            long long verify=0;if(!ReadAttr(phy_tx,"sampling_frequency",verify)||verify!=static_cast<long long>(kRate))
                throw std::runtime_error("TX sample rate did not become 15.36 MS/s");
            if(have_rx_rate){long long after=0;if(!ReadAttr(phy_rx,"sampling_frequency",after)||after!=rx_rate)
                throw std::runtime_error("TX rate setup changed RX sample rate; stopping TX");}
        }
        const auto lo_for=[&](uint64_t channel){return static_cast<long long>(channel-signed_offset);};
        int ret=WriteAttr(lo,"frequency",lo_for(o.hops[0]));
        if(ret>=0)ret=WriteAttr(phy_tx,"rf_bandwidth",static_cast<long long>(o.bandwidth_hz));
        char gain[32];std::snprintf(gain,sizeof(gain),"%.2f",o.gain_db);
        if(ret>=0)ret=WriteAttr(phy_tx,"hardwaregain",gain);
        if(ret<0)throw std::runtime_error("failed to configure Pluto TX");

        iio_channel_enable(i_chan);iio_channel_enable(q_chan);channels_enabled=true;
        buffer=iio_device_create_buffer(tx,wave.size(),false);if(!buffer)throw std::runtime_error("cannot create TX IIO buffer");
        const ptrdiff_t step=iio_buffer_step(buffer);char *ip=static_cast<char*>(iio_buffer_first(buffer,i_chan));char *qp=static_cast<char*>(iio_buffer_first(buffer,q_chan));
        if(step<=0||!ip||!qp)throw std::runtime_error("invalid TX IIO buffer layout");
        const char *buffer_start=static_cast<const char *>(iio_buffer_start(buffer));
        const char *buffer_end=static_cast<const char *>(iio_buffer_end(buffer));
        const size_t expected_bytes=static_cast<size_t>(buffer_end-buffer_start);
        for(size_t n=0;n<wave.size();++n){int16_t si=iq[2*n],sq=iq[2*n+1];iio_channel_convert_inverse(i_chan,ip+n*step,&si);iio_channel_convert_inverse(q_chan,qp+n*step,&sq);}

        const uint64_t frame_us=static_cast<uint64_t>(std::ceil(1e6*wave.size()/kRate));
        if(static_cast<uint64_t>(o.period_ms)*1000<frame_us)throw std::runtime_error("period is shorter than packet frame");
        std::cout<<"Pluto TX"<<o.tx_channel<<" ready: 15.36 MS/s"<<(current_rate==static_cast<long long>(kRate)?" (existing TX rate retained)":" (TX rate configured; RX rate verified unchanged)")
                 <<", bandwidth="<<o.bandwidth_hz<<" Hz gain="<<o.gain_db<<" dB frame="<<wave.size()<<" samples ("<<frame_us<<" us)\n"
                 <<"Offset="<<o.side<<" "<<o.offset_hz<<" Hz\nHops:";
        for(uint64_t f:o.hops)std::cout<<" "<<std::fixed<<std::setprecision(3)<<f/1e6<<"MHz";
        std::cout<<"\nPayload: id='"<<o.aircraft_id<<"' lat="<<o.latitude<<" lon="<<o.longitude<<" alt="<<o.altitude_m<<"m sequence="<<o.sequence<<"\n"<<std::flush;
        uint32_t sent=0;size_t hop=0;
        while(!g_stop&&(!o.count||sent<o.count)){
            const auto start=std::chrono::steady_clock::now();
            if(hop){ret=WriteAttr(lo,"frequency",lo_for(o.hops[hop]));if(ret<0)throw std::runtime_error("TX LO retune failed");}
            const ssize_t n=iio_buffer_push(buffer);if(n<0){char e[128];iio_strerror(static_cast<int>(-n),e,sizeof(e));throw std::runtime_error(std::string("IIO TX push failed: ")+e);}
            if(static_cast<size_t>(n)!=expected_bytes)throw std::runtime_error("IIO accepted a partial TX buffer");
            ++sent;std::cout<<"TX packet="<<sent<<" channel="<<o.hops[hop]<<" Hz samples="<<wave.size()<<" pushed="<<n<<" bytes\n"<<std::flush;
            hop=(hop+1)%o.hops.size();std::this_thread::sleep_until(start+std::chrono::milliseconds(o.period_ms));
        }
        std::cout<<"TX stopped after "<<sent<<" packet(s)\n";
    } catch(const std::exception &e){std::cerr<<"dji-droneid-tx: "<<e.what()<<"\n";if(buffer)iio_buffer_destroy(buffer);if(channels_enabled){iio_channel_disable(i_chan);iio_channel_disable(q_chan);}if(ctx)iio_context_destroy(ctx);return 1;}
    if(buffer)iio_buffer_destroy(buffer);if(channels_enabled){iio_channel_disable(i_chan);iio_channel_disable(q_chan);}if(ctx)iio_context_destroy(ctx);return 0;
}
