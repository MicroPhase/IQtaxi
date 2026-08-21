#include "MicroSDRUtil.hpp"

static bool is_supported_device_name(const char *name)
{
    return strcmp(name, MICROPHASE_NAME_E100) == 0 ||
           strcmp(name, MICROPHASE_NAME_E200) == 0 ||
           strcmp(name, MICROPHASE_NAME_E206) == 0;
}

struct if_addrs_t {
    std::string inet;   // 本机 IP
    std::string bcast;  // 广播地址
};

std::vector<if_addrs_t> get_if_addrs()
{
    std::vector<if_addrs_t> result;

    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return result;
    }

    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {

        if (!ifa->ifa_addr)
            continue;

        // 只要 IPv4
        if (ifa->ifa_addr->sa_family != AF_INET)
            continue;

        // 接口必须 UP
        if (!(ifa->ifa_flags & IFF_UP))
            continue;

        sockaddr_in* addr = (sockaddr_in*)ifa->ifa_addr;
        sockaddr_in* netmask = (sockaddr_in*)ifa->ifa_netmask;

        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr->sin_addr, ip, sizeof(ip));

        // 跳过 loopback
        if (strcmp(ip, "127.0.0.1") == 0)
            continue;

        // 计算广播地址
        uint32_t ip_u   = ntohl(addr->sin_addr.s_addr);
        uint32_t mask_u = ntohl(netmask->sin_addr.s_addr);
        uint32_t bcast_u = ip_u | ~mask_u;

        in_addr bcast_addr{};
        bcast_addr.s_addr = htonl(bcast_u);

        char bcast[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &bcast_addr, bcast, sizeof(bcast));

        result.push_back({
            ip,
            bcast
        });
    }

    freeifaddrs(ifaddr);
    return result;
}

KwargsList discover_device(int timeout_ms, unit_t &out_dev)
{
    KwargsList result;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return result;

    fcntl(sock, F_SETFL, O_NONBLOCK);

    int enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port   = htons(LISTEN_PORT); 
    local.sin_addr.s_addr = htonl(INADDR_ANY); 
    if (bind(sock, (struct sockaddr*)&local, sizeof(local)) < 0) {
        perror("bind discovery socket");
    }

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(LISTEN_PORT);
    
    unit_t send_data{};
    strncpy(send_data.check,MICROPHASE_CHECK,sizeof(send_data.check));
    strncpy(send_data.name,MICROPHASE_NAME_BR0,sizeof(send_data.name));
    uint8_t send_buf[sizeof(unit_t)] = {0};
    memcpy(send_buf,(uint8_t *)&send_data,sizeof(unit_t));
    std::set<std::string> local_ips;
    std::set<std::string> seen_devices;
    for(const if_addrs_t& if_addrs : get_if_addrs()){
        local_ips.insert(if_addrs.inet);
        inet_pton(AF_INET, if_addrs.bcast.c_str(), &dest.sin_addr);
        sendto(sock, send_buf, sizeof(send_buf), 0,
        (sockaddr *)&dest, sizeof(dest));
    }
    inet_pton(AF_INET, "255.255.255.255", &dest.sin_addr);
    sendto(sock, send_buf, sizeof(send_buf), 0,
    (sockaddr *)&dest, sizeof(dest));
    inet_pton(AF_INET, "192.168.1.10", &dest.sin_addr);
    sendto(sock, send_buf, sizeof(send_buf), 0,
    (sockaddr *)&dest, sizeof(dest));

    fd_set rfds;
    auto start = std::chrono::steady_clock::now();

    while(true){
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);

        timeval tv{};
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(sock + 1, &rfds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            close(sock);
            return result;  
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
            > (timeout_ms*10)) {
            break;
        }

        sockaddr_in from{};
        socklen_t len = sizeof(from);
        ssize_t n = recvfrom(sock, &out_dev, sizeof(out_dev), 0,
                            (sockaddr *)&from, &len);
        
        if(n <= 0)
            continue;

        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));

        if (local_ips.count(ip))
            continue;

        if (n == sizeof(unit_t) 
        and is_supported_device_name(out_dev.name)
        and strcmp(out_dev.check,MICROPHASE_CHECK) == 0){
            from.sin_port = htons(LISTEN_PORT);
            sendto(sock, send_buf, sizeof(send_buf), 0,
                   (sockaddr *)&from, sizeof(from));

            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);
            tv.tv_sec  = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            ret = select(sock + 1, &rfds, nullptr, nullptr, &tv);
            if (ret <= 0) {
                continue;
            }
            len = sizeof(from);
            n = recvfrom(sock, &out_dev, sizeof(out_dev), 0,
                         (sockaddr *)&from, &len);
            if (n == sizeof(unit_t) 
            and is_supported_device_name(out_dev.name)
            and strcmp(out_dev.check,MICROPHASE_CHECK) == 0){
                inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
                const std::string device_key = std::string(ip) + ":" + out_dev.name;
                if (seen_devices.count(device_key))
                    continue;
                seen_devices.insert(device_key);
                Kwargs candiate;
                std::string serial_str = "";
                for(int i=0;i<8;i++)
                    serial_str += std::to_string((int)out_dev.serial_number[i]);
                std::string board_str = "";
                for(int i=0;i<8;i++)
                    board_str += std::to_string((int)out_dev.board_version[i]);
                candiate["addr"] = ip;
                candiate["name"] = out_dev.name;
                candiate["serial"] = serial_str;    
                candiate["board"] = board_str;  
                result.push_back(candiate);
            }
        }
    }

    close(sock);
    return result;
}

int find_device(){
    unit_t unit;
    const auto results = discover_device(250,unit);

    for(int i=0;i<results.size();i++){
        std::cout << "find device " << i << std::endl;
        for(const auto &it:results[i]){
            std::cout << " " << it.first << ": " << it.second << std::endl;
        } 
        std::cout << std::endl;
    }

    return results.empty()? EXIT_FAILURE:EXIT_SUCCESS;
}

static void printBannner(void){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "---  MicroPhase SDR Utility v1.0.0  ---" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
}

static int printHelp(void){
    std::cout << "Usage MicroSDRUtil [options]" << std::endl;
    std::cout << "  Options summary:" << std::endl;
    std::cout << "    --find[=\"driver=foo,type=bar\"] \t Discover available devices" << std::endl;


    std::cout << std::endl;
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[]){
    std::string serial;
    std::string argStr;
    std::string formatStr;
    std::string chanStr;
    std::string dirStr;
    double sampleRate(0.0);
    std::string driverName;
    bool findDevicesFlag(false);

    static struct option long_options[] = {
        {"help", no_argument, nullptr, 'h'},
        {"find", optional_argument, nullptr, 'f'},
        // {"make", optional_argument, nullptr, 'm'},
        // {"info", optional_argument, nullptr, 'i'},
        // {"probe", optional_argument, nullptr, 'p'},
        // {"watch", optional_argument, nullptr, 'w'},

        // {"check", optional_argument, nullptr, 'c'},
        // {"sparse", no_argument, nullptr, 's'},
        // {"serial", required_argument, nullptr, 'S'},

        // {"args", optional_argument, nullptr, 'a'},
        // {"rate", optional_argument, nullptr, 'r'},
        // {"format", optional_argument, nullptr, 't'},
        // {"channels", optional_argument, nullptr, 'n'},
        // {"direction", optional_argument, nullptr, 'd'},
        {nullptr, no_argument, nullptr, '\0'}
    };

    int long_index = 0;
    int option = 0;
    while((option = getopt_long_only(argc,argv,"",long_options,&long_index)) != -1){
        switch(option){
            case 'h':
                return printHelp();
            case 'f':
                findDevicesFlag = true;
                if(optarg != nullptr) argStr = optarg;
                break;
            default:
                break;
        }
    }
    printBannner();

    if(findDevicesFlag) return find_device();
}
