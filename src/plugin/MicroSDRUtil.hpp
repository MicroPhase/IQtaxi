#include <string>
#include <fcntl.h>          // fcntl(), O_NONBLOCK
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <memory>
#include <cstring>
#include <chrono>
#include <set>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <map>
#include "include/sdr/api/UdpDiscover.hpp"

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <ifaddrs.h>
    #include <net/if.h>
#endif

#include <errno.h>

typedef std::map<std::string,std::string> Kwargs;
typedef std::vector<Kwargs> KwargsList;

#define LISTEN_PORT 49100
#define MICROPHASE_CHECK "MicroPhase"

#define    MICROPHASE_NAME_BR0 "e100"
#define    MICROPHASE_NAME_E100 "E100"
#define    MICROPHASE_NAME_E200 "E200"
#define    MICROPHASE_NAME_E206 "E206"

using unit_t = sdr::api::IqtaxiUdpDiscoverPacket;
