#include "iqtaxi_impl.hpp"
#include "iqtaxi_rx_streamer.hpp"
#include "iqtaxi_tx_streamer.hpp"
#include "uhd/transport/if_addrs.hpp"
#include "uhd/transport/udp_simple.hpp"
#include "uhd/utils/byteswap.hpp"
#include "../../driver/M300/m300_xdma_discovery.hpp"
#include <algorithm>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/foreach.hpp>
#include <cctype>
#include <cmath>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

namespace {

bool hint_requests_m300(const device_addr_t& hint)
{
    auto matches = [](const std::string& value) {
        return value == "M300" || value == "m300" ||
               value == "M300_XDMA" || value == "m300_xdma";
    };
    if (hint.has_key("type") && matches(hint["type"])) return true;
    if (hint.has_key("product") && matches(hint["product"])) return true;
    if (hint.has_key("name") && matches(hint["name"])) return true;
    if (hint.has_key("device") && matches(hint["device"])) return true;
    if (hint.has_key("driver") && hint["driver"] == "M300_XDMA") return true;
    return hint.has_key("addr") && hint["addr"].rfind("/dev/xdma", 0) == 0;
}

bool hint_allows_m300(const device_addr_t& hint)
{
    if (hint_requests_m300(hint)) {
        return true;
    }
    if (hint.has_key("type")) {
        const std::string type = hint["type"];
        if (type != "iqtaxi" && type != "ant" && type != "microphase") {
            return false;
        }
    }
    if (hint.has_key("product") || hint.has_key("name") ||
        hint.has_key("device") || hint.has_key("driver") || hint.has_key("addr")) {
        return false;
    }
    return true;
}

device_addrs_t find_m300(const device_addr_t& hint)
{
    std::vector<std::string> candidates;
    if (hint.has_key("addr")) {
        candidates.push_back(sdr::driver::normalize_m300_xdma_base(hint["addr"]));
    } else {
        candidates = sdr::driver::enumerate_m300_xdma_candidates();
    }

    device_addrs_t found_devices;
    for (const std::string& addr : candidates) {
        sdr::driver::m300_discovery_info info;
        std::string error;
        if (!sdr::driver::probe_m300_xdma(addr, &info, &error)) {
            UHD_LOGGER_DEBUG("Microphase")
                << "Ignoring XDMA candidate " << addr << ": " << error;
            continue;
        }
        if (hint.has_key("serial") && hint["serial"] != info.serial) {
            continue;
        }

        device_addr_t found;
        found["type"] = "m300";
        found["driver"] = "M300_XDMA";
        found["product"] = MICROPHASE_NAME_M300;
        found["name"] = "Microphase M300";
        found["addr"] = info.addr;
        found["serial"] = info.serial;
        found["protocol"] = std::to_string(info.protocol_version);
        found["capabilities"] = std::to_string(info.capabilities);
        if (!info.pci_bdf.empty()) {
            found["pci_bdf"] = info.pci_bdf;
        }
        found_devices.push_back(found);
    }
    return found_devices;
}

} // namespace

std::string check_e100_option_valid(const std::string& name,
    const std::vector<std::string>& valid_options,
    const std::string& option)
{
    if (std::find(valid_options.begin(), valid_options.end(), option)
        == valid_options.end()) {
        throw uhd::runtime_error(
            str(boost::format("Invalid option chosen for: %s") % name));
    }

    return option;
}

uint64_t str_to_uint64(const char* str) {
    uint64_t result = 0;
    size_t len = strlen(str);

    for (size_t i = 0; i < len && i < 8; ++i) {
        result |= (static_cast<uint64_t>(str[i]) << (8 * i));
    }

    return result;
}

static bool is_supported_microphase_device(const char* name)
{
    return strcmp(name, MICROPHASE_NAME_E100) == 0 ||
           strcmp(name, MICROPHASE_NAME_E200) == 0 ||
           strcmp(name, MICROPHASE_NAME_E206) == 0;
}

static bool is_e200_product(const std::string& product)
{
    return product == MICROPHASE_NAME_E200 || product == "ANTSDR-E200";
}

static bool is_e206_product(const std::string& product)
{
    return product == MICROPHASE_NAME_E206 ||
           product == "ANTSDR-E206";
}

static bool hint_accepts_microphase_device(
    const uhd::device_addr_t& hint,
    const char* device_name)
{
    const std::string name(device_name);

    if (hint.has_key("type")) {
        const std::string type = hint["type"];
        if (type != "iqtaxi" && type != "ant" && type != "microphase" &&
            type != "E100" && type != "e100" &&
            type != "E200" && type != "e200" &&
            type != "E206" && type != "e206") {
            return false;
        }
        if ((type == "E100" || type == "e100") && name != MICROPHASE_NAME_E100) {
            return false;
        }
        if ((type == "E200" || type == "e200") && name != MICROPHASE_NAME_E200) {
            return false;
        }
        if ((type == "E206" || type == "e206") && name != MICROPHASE_NAME_E206) {
            return false;
        }
    }

    if (hint.has_key("product")) {
        const std::string product = hint["product"];
        if (product != name && product != ("ANTSDR-" + name) &&
            !(is_e206_product(product) && name == MICROPHASE_NAME_E206)) {
            return false;
        }
    }

    return true;
}

static std::string get_explicit_product_from_hint(const uhd::device_addr_t& hint)
{
    if (hint.has_key("type")) {
        const std::string type = hint["type"];
        if (type == "E200" || type == "e200") {
            return MICROPHASE_NAME_E200;
        }
        if (type == "E206" || type == "e206") {
            return MICROPHASE_NAME_E206;
        }
        if (type == "E100" || type == "e100") {
            return MICROPHASE_NAME_E100;
        }
    }

    if (hint.has_key("product")) {
        const std::string product = hint["product"];
        if (product == MICROPHASE_NAME_E200 || product == "ANTSDR-E200") {
            return MICROPHASE_NAME_E200;
        }
        if (is_e206_product(product)) {
            return MICROPHASE_NAME_E206;
        }
        if (product == MICROPHASE_NAME_E100 || product == "ANTSDR-E100") {
            return MICROPHASE_NAME_E100;
        }
    }

    if (hint.has_key("name")) {
        const std::string name = hint["name"];
        if (name == MICROPHASE_NAME_E200 || name == "ANTSDR-E200") {
            return MICROPHASE_NAME_E200;
        }
        if (is_e206_product(name)) {
            return MICROPHASE_NAME_E206;
        }
        if (name == MICROPHASE_NAME_E100 || name == "ANTSDR-E100") {
            return MICROPHASE_NAME_E100;
        }
    }

    if (hint.has_key("device")) {
        const std::string device = hint["device"];
        if (device == MICROPHASE_NAME_E200) {
            return MICROPHASE_NAME_E200;
        }
        if (device == MICROPHASE_NAME_E206) {
            return MICROPHASE_NAME_E206;
        }
        if (device == MICROPHASE_NAME_E100) {
            return MICROPHASE_NAME_E100;
        }
    }

    if (hint.has_key("driver")) {
        const std::string driver = hint["driver"];
        if (driver == "E200_UDP") {
            return MICROPHASE_NAME_E200;
        }
        if (driver == "E206_UDP") {
            return MICROPHASE_NAME_E206;
        }
        if (driver == "E100_UDP") {
            return MICROPHASE_NAME_E100;
        }
    }

    return "";
}

//这里的hint是应用传下来的“设备查找条件”，类似于uhd_find_device --args "type=ant"
static device_addrs_t iqtaxi_find(const device_addr_t& hint)
{
    device_addrs_t hints = separate_device_addr(hint);

    if(hints.size() > 1){
        device_addrs_t found_devices;
        std::string error_msg;
        for(const device_addr_t& hint_i : hints){
            device_addrs_t found_device_i = iqtaxi_find(hint_i);
            //给的多个设备条件里都必须找到一个设备
            if(found_device_i.size() != 1)
                error_msg +=
                        str(boost::format(
                                "Could not resolve device hint \"%s\" to a single device.")
                            % hint_i.to_string());
            else
                found_devices.push_back(found_device_i[0]);
        }
        if(found_devices.empty())
            return device_addrs_t();
        if(not error_msg.empty())
            throw uhd::value_error(error_msg);
        return device_addrs_t(1, combine_device_addrs(found_devices));
    }

    // initialize the hint for a single device case
    UHD_ASSERT_THROW(hints.size() <= 1);
    hints.resize(1);
    device_addr_t hint_ = hints[0];
    device_addrs_t e100_addrs;

    if (hint_requests_m300(hint_)) {
        return find_m300(hint_);
    }

    if (hint_allows_m300(hint_)) {
        e100_addrs = find_m300(hint_);
    }

    if (hint_.has_key("type")) {
        const std::string type = hint_["type"];
        if (type != "iqtaxi" && type != "ant" && type != "microphase" &&
            type != "E100" && type != "e100" &&
            type != "E200" && type != "e200" &&
            type != "E206" && type != "e206") {
            return e100_addrs;
        }
    }

    if (hint_.has_key("product")) {
        const std::string product = hint_["product"];
        if (product != MICROPHASE_NAME_E100 &&
            product != MICROPHASE_NAME_E200 &&
            product != MICROPHASE_NAME_E206 &&
            product != "ANTSDR-E100" &&
            product != "ANTSDR-E200" &&
            product != "ANTSDR-E206") {
            return e100_addrs;
        }
    }

    if (hint_.has_key("name")) {
        const std::string name = hint_["name"];
        if (name != MICROPHASE_NAME_E100 &&
            name != MICROPHASE_NAME_E200 &&
            name != MICROPHASE_NAME_E206 &&
            name != "ANTSDR-E100" &&
            name != "ANTSDR-E200" &&
            name != "ANTSDR-E206") {
            return e100_addrs;
        }
    }

    if (hint_.has_key("device")) {
        const std::string device = hint_["device"];
        if (device != MICROPHASE_NAME_E100 &&
            device != MICROPHASE_NAME_E200 &&
            device != MICROPHASE_NAME_E206) {
            return e100_addrs;
        }
    }

    if (hint_.has_key("driver")) {
        const std::string driver = hint_["driver"];
        if (driver != "IQTAXI" && driver != "E100_UDP" && driver != "E200_UDP" &&
            driver != "E206_UDP") {
            return e100_addrs;
        }
    }

    if(not hint_.has_key("addr")){
        for(const if_addrs_t& if_addrs : get_if_addrs()){
            if(if_addrs.inet == boost::asio::ip::address_v4::loopback().to_string())
                continue;

            device_addr_t new_hint = hint;
            new_hint["addr"] = if_addrs.bcast;

            device_addrs_t new_e100_addrs = iqtaxi_find(new_hint);
            e100_addrs.insert(
                    e100_addrs.begin(),new_e100_addrs.begin(),new_e100_addrs.end());
        }
        return e100_addrs;
    }
    /* connect the device from ethernet "addr=" */
    udp_simple::sptr udp_transport;
    try {
        udp_transport = udp_simple::make_broadcast(
                hint_["addr"], BOOST_STRINGIZE(MICROPHASE_IQTAXI_UDP_FIND_PORT));
    } catch (const std::exception &e) {
        UHD_LOGGER_ERROR("Microphase")
                << "Cannot open UDP transport on " << hint_["addr"] << ":" << e.what();
        return e100_addrs;
    }

    microphase_e100_ctrl_data_t ctrl_data_out = microphase_e100_ctrl_data_t();
    strncpy(ctrl_data_out.check,MICROPHASE_CHECK,sizeof(ctrl_data_out.check));
    strncpy(ctrl_data_out.name,MICROPHASE_NAME_BR0,sizeof(ctrl_data_out.name));
    try {
        udp_transport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
    } catch (const std::exception &ex) {
        UHD_LOGGER_ERROR("Microphase ANT") << "ANT Network discovery error" << ex.what();
    } catch (...) {
        UHD_LOGGER_ERROR("Microphase ANT") << "ANT Network discovery unkonwn error";
    }
    //loop and recieve until the timeout
    uint8_t microphase_e100_ctrl_data_in_mem[udp_simple::mtu];

    const microphase_e100_ctrl_data_t *ctrl_data_in =
            reinterpret_cast<const microphase_e100_ctrl_data_t *>(microphase_e100_ctrl_data_in_mem);

    while (true) {
        size_t len = udp_transport->recv(boost::asio::buffer(microphase_e100_ctrl_data_in_mem));

        if (len >= sizeof(ctrl_data_out)
            and strcmp(ctrl_data_in->check,MICROPHASE_CHECK) == 0
            and is_supported_microphase_device(ctrl_data_in->name)
            and hint_accepts_microphase_device(hint_, ctrl_data_in->name)) {
            // make a boost asio ipv4 with the raw addr in host byte order1

            device_addr_t mp_addr;
            mp_addr["type"] = "iqtaxi";
            mp_addr["legacy_type"] = "ant";
            mp_addr["addr"] = udp_transport->get_recv_addr();

            udp_simple::sptr ctrl_xport = udp_simple::make_connected(
                    mp_addr["addr"], BOOST_STRINGIZE(MICROPHASE_IQTAXI_UDP_FIND_PORT)
            );
            // Query the discovered device through its connected unicast
            // transport. Reusing the broadcast transport here makes it
            // receive its own request first; the subsequent `continue`
            // leaves the device reply queued in the outer loop and creates
            // an endless request/reply discovery cycle when two E200s are
            // online concurrently.
            ctrl_xport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
            size_t len = ctrl_xport->recv(boost::asio::buffer(microphase_e100_ctrl_data_in_mem));
            if (len >= sizeof(ctrl_data_out)
                and strcmp(ctrl_data_in->check,MICROPHASE_CHECK) == 0
                and is_supported_microphase_device(ctrl_data_in->name)
                and hint_accepts_microphase_device(hint_, ctrl_data_in->name)) {
                std::string serial_str = "";
                for(int i=0;i<32;i++)
                    serial_str += std::to_string((int)ctrl_data_in->serial_number[i]);
                std::string board_str = "";
                for(int i=0;i<8;i++)
                    board_str += std::to_string((int)ctrl_data_in->board_version[i]);
                mp_addr["product"] = ctrl_data_in->name;
                mp_addr["serial"] = serial_str.substr(0,8);
                mp_addr["name"] = "ANTSDR-" + std::string(ctrl_data_in->name);
                std::cout << mp_addr["serial"] << std::endl;
                // found the device,open up for communication!
                e100_addrs.push_back(mp_addr);
            } else {
                continue;
            }
        }
        if (len == 0)
            break;
    }

    const std::string explicit_product = get_explicit_product_from_hint(hint_);
    if (e100_addrs.empty() && !explicit_product.empty()) {
        device_addr_t mp_addr;
        mp_addr["type"] = "iqtaxi";
        mp_addr["legacy_type"] = "ant";
        mp_addr["addr"] = hint_["addr"];
        mp_addr["product"] = explicit_product;
        mp_addr["name"] = "ANTSDR-" + explicit_product;
        if (hint_.has_key("serial")) {
            mp_addr["serial"] = hint_["serial"];
        }
        UHD_LOGGER_WARNING("Microphase")
            << "No discovery response from " << hint_["addr"]
            << "; using explicit " << explicit_product << " device hint";
        e100_addrs.push_back(mp_addr);
    }

    return e100_addrs;
}

static device::sptr iqtaxi_make(const device_addr_t& device_addr){
    try{
        return device::sptr(new iqtaxi_impl(device_addr));
    }catch(const uhd::assertion_error&) {
        UHD_LOGGER_INFO("ANT") << "Detetcted ANT net state; resetting device";
    }

    return device::sptr(new iqtaxi_impl(device_addr));
}

UHD_STATIC_BLOCK(register_iqtaxi_device){
    device::register_device(&iqtaxi_find,&iqtaxi_make,device::USRP);
}

iqtaxi_impl::iqtaxi_impl(const uhd::device_addr_t &device_addr){
    _tree = property_tree::make();
    _type = device::USRP;
    _tick_rate = defaultClockRate;

    _time_source = UNKNOWN;
    const std::string initial_clock_source = device_addr.has_key("clock_source")
        ? device_addr["clock_source"]
        : "internal";
    if (device_addr.has_key("ignore_tx_timestamps")) {
        std::string value = device_addr["ignore_tx_timestamps"];
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (value == "0" || value == "false" || value == "no" || value == "off") {
            _ignore_tx_timestamps = false;
        } else if (value == "1" || value == "true" || value == "yes" || value == "on") {
            _ignore_tx_timestamps = true;
        } else {
            throw uhd::value_error(
                "ignore_tx_timestamps must be true/false, yes/no, on/off, or 1/0");
        }
    }
    const fs_path mb_path = "/mboards/0";

    if (device_addr.has_key("type") &&
        (device_addr["type"] == "ant" ||
         device_addr["type"] == "iqtaxi" ||
         device_addr["type"] == "microphase" ||
         device_addr["type"] == "E100" ||
         device_addr["type"] == "e100" ||
         device_addr["type"] == "E200" ||
         device_addr["type"] == "e200" ||
         device_addr["type"] == "E206" ||
         device_addr["type"] == "e206" ||
         device_addr["type"] == "M300" ||
         device_addr["type"] == "m300" ||
         device_addr["type"] == "M300_XDMA" ||
         device_addr["type"] == "m300_xdma")) {
        const std::string addr = device_addr.has_key("addr") ? device_addr["addr"] : "/dev/xdma0";
        std::string product_name = device_addr.has_key("product") ? device_addr["product"] : "E100";
        if (device_addr["type"] == "E200" || device_addr["type"] == "e200") {
            product_name = MICROPHASE_NAME_E200;
        } else if (device_addr["type"] == "E206" || device_addr["type"] == "e206") {
            product_name = MICROPHASE_NAME_E206;
        } else if (device_addr["type"] == "E100" || device_addr["type"] == "e100") {
            product_name = MICROPHASE_NAME_E100;
        }
        _is_m300 = product_name == MICROPHASE_NAME_M300 ||
                   device_addr["type"] == "M300" || device_addr["type"] == "m300" ||
                   device_addr["type"] == "M300_XDMA" || device_addr["type"] == "m300_xdma";
        if (_is_m300) {
            product_name = MICROPHASE_NAME_M300;
            _tick_rate = 61.44e6;
        } else if (is_e206_product(product_name)) {
            product_name = MICROPHASE_NAME_E206;
        }
        const bool use_e200_backend = is_e200_product(product_name);
        const bool use_e206_backend = is_e206_product(product_name);
        const std::string backend_name = _is_m300 ? "M300_XDMA" :
            (use_e200_backend ? MICROPHASE_NAME_E200 :
             (use_e206_backend ? MICROPHASE_NAME_E206 : MICROPHASE_NAME_E100));

        UHD_LOGGER_INFO("ANT") << "Detected Device: ANTSDR-" << product_name;
        UHD_LOGGER_INFO("ANT") << "TX timestamp scheduling: "
                                << (_ignore_tx_timestamps ? "ignored (ASAP)" : "strict");

        mboard_eeprom_t mb_eeprom;
        mb_eeprom["magic"] = "45568";
        mb_eeprom["eeprom_revision"] = "v0.1";
        mb_eeprom["eeprom_compat"] = "1";
        mb_eeprom["product"] = "MicroPhase";
        mb_eeprom["name"] = "ANTSDR-" + product_name;
        mb_eeprom["serial"] = device_addr.has_key("serial") ? device_addr["serial"] : "";
        _tree->create<mboard_eeprom_t>(mb_path / "eeprom")
                .set(mb_eeprom)
                .add_coerced_subscriber(
                        std::bind(&iqtaxi_impl::set_mb_eeprom, this)
                );
        std::cout << product_name << " UHD impl init" << std::endl;

        zero_copy_xport_params default_buff_args;
        default_buff_args.send_frame_size = transport::udp_simple::mtu;
        default_buff_args.recv_frame_size = transport::udp_simple::mtu;
        default_buff_args.num_send_frames = 16;
        default_buff_args.num_recv_frames = 16;
        if(not device_addr.has_key("recv_buff_size")){
            default_buff_args.recv_buff_size = 1e6;
        }
        if(not device_addr.has_key("send_buff_size")){
            default_buff_args.send_buff_size = 1e6;
        }
        
        /* make the transprt object with the hintS
        * create the transport port (_ctrl_transport)
        * */
        device_addr_t filtered_hints = device_addr;

        iqtaxi_device = sdr::api::Device::makeDevice(backend_name, addr);
        if (!iqtaxi_device) {
            throw uhd::runtime_error(
                str(boost::format("failed to create IQTAXI %s backend at %s") %
                    backend_name % addr));
        }
        _profile = &iqtaxi_device->get_profile();
        //uoe
        iqtaxi_device->set_dma_mode(0);
        
        if (auto udp_device = std::dynamic_pointer_cast<IqtaxiUdpImpl>(iqtaxi_device)) {
            _local_bus = udp_device->get_local_bus();
            _rx_stream_bus = udp_device->get_rx_stream_bus();
            _tx_stream_bus = udp_device->get_tx_stream_bus();
        }
        ////////////////////////////////////////////////////////////////////
        // Initialize the properties tree
        ////////////////////////////////////////////////////////////////////
        _tree->create<std::string>("/name").set(
            _is_m300 ? "Microphase M300" : "AntSDR-E-Series Device");
        _tree->create<std::string>(mb_path / "name").set(product_name);
        _tree->create<std::string>(mb_path / "codename").set( "HuangPu River");


        // ////////////////////////////////////////////////////////////////////
        // // create codec control objects
        // ////////////////////////////////////////////////////////////////////
        {
            const fs_path codec_path = mb_path / ("rx_codecs") / "A";
            _tree->create<std::string>(codec_path / "name").set(product_name + " RX ADC");
            _tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
        }
        {
            const fs_path codec_path = mb_path / ("tx_codecs") / "A";
            _tree->create<std::string>(codec_path / "name").set(product_name + " TX DAC");
            _tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
        }

        ////////////////////////////////////////////////////////////////////
        // create clock control objects
        ////////////////////////////////////////////////////////////////////
        _tree->create<double>(mb_path / "tick_rate")
                .set_coercer(std::bind(&iqtaxi_impl::set_tick_rate, this, std::placeholders::_1))
                .set_publisher(std::bind(&iqtaxi_impl::get_tick_rate, this))
                .add_coerced_subscriber(
                        std::bind(&iqtaxi_impl::update_tick_rate, this, std::placeholders::_1));
        _tree->create<meta_range_t>(mb_path / "tick_rate/range").set_publisher([this]() {
            return get_sample_range();
        });
        _tree->create<uhd::time_spec_t>(mb_path / "time" / "cmd");
        _tree->create<bool>(mb_path / "auto_tick_rate").set(false);


        // ////////////////////////////////////////////////////////////////////
        // // and do the misc mboard sensors
        // ////////////////////////////////////////////////////////////////////
        _tree->create<sensor_value_t>(mb_path / "sensors" / "ref_locked")
                .set_publisher(std::bind(&iqtaxi_impl::get_ref_locked, this));

        UHD_LOGGER_INFO("ANT") << "after  do the misc mboard sensors";

        // ////////////////////////////////////////////////////////////////////
        // // create frontend mapping
        // ////////////////////////////////////////////////////////////////////
        // std::vector<size_t> default_map(2, 0);
        // default_map[1] = 1; // Set this to A->0 B->1 even if there's only A
        // _tree->create<std::vector<size_t>>(mb_path / "rx_chan_dsp_mapping").set(default_map);
        // _tree->create<std::vector<size_t>>(mb_path / "tx_chan_dsp_mapping").set(default_map);
        // UHD_LOGGER_INFO("ANT") << "after ================== tx_chan_dsp_mapping ...";
        // _tree->create<subdev_spec_t>(mb_path / "rx_subdev_spec")
        //         .set_coercer(
        //                 std::bind(&iqtaxi_impl::coerce_subdev_spec, this, std::placeholders::_1))
        //         .set(subdev_spec_t())
        //         .add_coerced_subscriber(
        //                 std::bind(&iqtaxi_impl::update_subdev_spec, this, "rx", std::placeholders::_1));
        // _tree->create<subdev_spec_t>(mb_path / "tx_subdev_spec")
        //         .set_coercer(
        //                 std::bind(&iqtaxi_impl::coerce_subdev_spec, this, std::placeholders::_1))
        //         .set(subdev_spec_t())
        //         .add_coerced_subscriber(
        //                 std::bind(&iqtaxi_impl::update_subdev_spec, this, "tx", std::placeholders::_1));
        
        
        std::vector<size_t> default_map(_is_m300 ? 1u : 2u, 0);
        if (!_is_m300) default_map[1] = 1;
        _tree->create<std::vector<size_t> >(mb_path / "rx_chan_dsp_mapping").set(default_map);

        _tree->create<std::vector<size_t> >(mb_path / "tx_chan_dsp_mapping").set(default_map);

        _rx_frontend_map.resize(default_map.size(), 0);
        
        _tree->create<uhd::usrp::subdev_spec_t>(mb_path / "rx_subdev_spec")
            .set_publisher(std::bind(&iqtaxi_impl::get_frontend_mapping, this, RX_DIRECTION))
            .set(subdev_spec_t())
            .add_coerced_subscriber(std::bind(&iqtaxi_impl::set_frontend_mapping, this, RX_DIRECTION, std::placeholders::_1));

        _tx_frontend_map.resize(default_map.size(), 0);
        _tree->create<uhd::usrp::subdev_spec_t>(mb_path / "tx_subdev_spec")
            .set_publisher(std::bind(&iqtaxi_impl::get_frontend_mapping, this, TX_DIRECTION))
            .set(subdev_spec_t())
            .add_coerced_subscriber(std::bind(&iqtaxi_impl::set_frontend_mapping, this, TX_DIRECTION, std::placeholders::_1));

            
        ////////////////////////////////////////////////////////////////////
        // setup radio control
        ////////////////////////////////////////////////////////////////////
        UHD_LOGGER_INFO("ANT") << "Initialize Radio control...";
        // confirm how many chanels e100 have, e100 only support 1r1t mode concurrently.
        // const size_t num_radio_chains = ((_local_ctrl->peek32(RB32_CORE_STATUS) >> 8) & 0xff);
        const size_t num_radio_chains = 1;
        UHD_ASSERT_THROW(num_radio_chains > 0);
        UHD_ASSERT_THROW(num_radio_chains <= 2);
        _radio_perifs.resize(num_radio_chains);

        for (size_t i = 0; i < _radio_perifs.size(); i++)
            this->setup_radio(i);

        // register time now and pps onto available radio cores
        _tree->create<uhd::time_spec_t>(mb_path / "time" / "now")
                .set_publisher(std::bind(&iqtaxi_impl::get_time_now, this))
                .add_coerced_subscriber(
                        std::bind(&iqtaxi_impl::set_time, this, std::placeholders::_1))
                .set(0.0);
        // re-sync the times when the tick rate changes
        _tree->access<double>(mb_path / "tick_rate")
                .add_coerced_subscriber(std::bind(&iqtaxi_impl::sync_times, this));
        _tree->create<uhd::time_spec_t>(mb_path / "time" / "pps")
                .set_publisher(
                        std::bind(&iqtaxi_impl::get_time_last_pps, this));
        for (radio_perifs_t &perif: _radio_perifs) {
            _tree->access<uhd::time_spec_t>(mb_path / "time" / "pps")
                    .add_coerced_subscriber(std::bind(
                            &iqtaxi_impl::set_time_next_pps, this, std::placeholders::_1));
        }
        UHD_LOGGER_INFO("ANT") << "register time now and pps onto available radio cores...";
        // setup time source props
        const std::vector<std::string> time_sources = std::vector<std::string>{"none", "internal", "external"};

        _tree->create<std::vector<std::string>>(mb_path / "time_source" / "options")
                .set(time_sources);
        _tree->create<std::string>(mb_path / "time_source" / "value")
                .set_coercer(std::bind(
                        &check_e100_option_valid, "time source", time_sources, std::placeholders::_1))
                .add_coerced_subscriber(
                        std::bind(&iqtaxi_impl::update_time_source, this, std::placeholders::_1));
        // setup reference source props
        const std::vector<std::string> clock_sources = std::vector<std::string>{"internal", "external"};
        _tree->create<std::vector<std::string>>(mb_path / "clock_source" / "options")
                .set(clock_sources);
        _tree->create<std::string>(mb_path / "clock_source" / "value")
                .set_coercer(std::bind(
                        &check_e100_option_valid, "clock source", clock_sources, std::placeholders::_1))
                .add_coerced_subscriber(
                        std::bind(&iqtaxi_impl::update_clock_source, this, std::placeholders::_1));

        UHD_LOGGER_INFO("ANT") << "setup time source props...";
        ////////////////////////////////////////////////////////////////////
        // front panel gpio
        ////////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////
        // dboard eeproms but not really
        ////////////////////////////////////////////////////////////////////
        dboard_eeprom_t db_eeprom;
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "rx_eeprom")
                .set(db_eeprom);
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "tx_eeprom")
                .set(db_eeprom);
        _tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "gdb_eeprom")
                .set(db_eeprom);

        ////////////////////////////////////////////////////////////////////
        // do some post-init tasks
        ////////////////////////////////////////////////////////////////////
        // Init the clock rate and the auto mcr appropriately
        // if (not device_addr.has_key("master_clock_rate")) {
        //     UHD_LOGGER_INFO("ANT") << "Setting master clock rate selection to 'automatic'.";
        // }
        // // We can automatically choose a master clock rate, but not if the user specifies one
        // const double default_tick_rate = 15.36e6;
        // _tree->access<double>(mb_path / "tick_rate").set(default_tick_rate);
        // _tree->access<bool>(mb_path / "auto_tick_rate")
        //         .set(not device_addr.has_key("master_clock_rate"));


        subdev_spec_t spec;
        spec.push_back(subdev_spec_pair_t("A", "A"));
        if (!_is_m300) spec.push_back(subdev_spec_pair_t("A", "A"));
        _tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(spec);
        _tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(spec);


        _tick_rate = _is_m300 ? 61.44e6 : defaultClockRate;
        _tree->access<double>(mb_path / "tick_rate").set(_tick_rate);

        //init to internal clock and time source
        _tree->access<std::string>(mb_path / "clock_source/value").set(initial_clock_source);
        _tree->access<std::string>(mb_path / "time_source/value").set("internal");
       
    }
}

iqtaxi_impl::~iqtaxi_impl(void)
{
    // UHD_SAFE_CALL(_async_task.reset();)
}

void iqtaxi_impl::setup_radio(const size_t dspno) {
	radio_perifs_t& perif = _radio_perifs[dspno];
    const fs_path mb_path = "/mboards/0";

    // perif.ctrl = _local_ctrl;
    // perif.ctrl->hold_task(_async_task);
    // _async_task_data->ctrl = perif.ctrl; // weak


    _tree->access<uhd::time_spec_t>(mb_path / "time" / "cmd")
        .add_coerced_subscriber(std::bind(
            &iqtaxi_impl::set_time, this, std::placeholders::_1));

    
    ////////////////////////////////////////////////////////////////////
    // Set up transport
    ////////////////////////////////////////////////////////////////////
    const uint32_t sid =  e100_CTRL0_MSG_SID ;
    const fs_path rx_dsp_path = mb_path / "rx_dsps" / dspno;

    _tree->create<uhd::meta_range_t>(rx_dsp_path / "rate" / "range")
		.set_publisher(std::bind(&iqtaxi_impl::get_sample_range, this));
	_tree->create<double>(rx_dsp_path / "rate" / "value")
		.set_publisher(std::bind(&iqtaxi_impl::getSampleRate, this, RX_DIRECTION, dspno))
		.add_coerced_subscriber(std::bind(&iqtaxi_impl::setSampleRate, this, RX_DIRECTION, dspno, std::placeholders::_1));
	//dsp freq
	_tree->create<double>(rx_dsp_path / "freq" / "value")
		.set_publisher(std::bind(&iqtaxi_impl::getFrequency, this, RX_DIRECTION, dspno, "BB"))
		.add_coerced_subscriber(std::bind(&iqtaxi_impl::set_frequency, this, RX_DIRECTION, "BB", std::placeholders::_1));
	_tree->create<uhd::meta_range_t>(rx_dsp_path / "freq" / "range")
		.set_publisher(std::bind(&iqtaxi_impl::getFrequencyRange, this, RX_DIRECTION, dspno, "BB"));
	_tree->create<uhd::stream_cmd_t>(rx_dsp_path / "stream_cmd")
		.add_coerced_subscriber(std::bind(&iqtaxi_impl::old_issue_stream_cmd, this, dspno, std::placeholders::_1));
    // _tree->create<stream_cmd_t>(rx_dsp_path / "stream_cmd")
    //     .add_coerced_subscriber(std::bind(&iqtaxi_rx_streamer::issue_stream_cmd,
    //         ,
    //         std::placeholders::_1));

    ////////////////////////////////////////////////////////////////////
	// create tx dsp control objects
	////////////////////////////////////////////////////////////////////
	const fs_path tx_dsp_path = mb_path / "tx_dsps" / dspno;

	_tree->create<uhd::meta_range_t>(tx_dsp_path / "rate" / "range")
		.set_publisher(std::bind(&iqtaxi_impl::get_sample_range, this));
	_tree->create<double>(tx_dsp_path / "rate" / "value")
		.set_publisher(std::bind(&iqtaxi_impl::getSampleRate, this, TX_DIRECTION, dspno))
		.add_coerced_subscriber(std::bind(&iqtaxi_impl::setSampleRate, this, TX_DIRECTION, dspno, std::placeholders::_1));
	//dsp freq
	_tree->create<double>(tx_dsp_path / "freq" / "value")
		.set_publisher(std::bind(&iqtaxi_impl::getFrequency, this, TX_DIRECTION, dspno, "BB"))
		.add_coerced_subscriber(std::bind(&iqtaxi_impl::set_frequency, this, TX_DIRECTION, "BB", std::placeholders::_1));
	_tree->create<uhd::meta_range_t>(tx_dsp_path / "freq" / "range")
		.set_publisher(std::bind(&iqtaxi_impl::getFrequencyRange, this, TX_DIRECTION, dspno, "BB"));

    ////////////////////////////////////////////////////////////////////
	// create RF frontend interfacing
	////////////////////////////////////////////////////////////////////
	static const std::vector<direction_t> dirs = boost::assign::list_of(RX_DIRECTION)(TX_DIRECTION);
	BOOST_FOREACH(direction_t dir, dirs) {

		const std::string x = (dir == RX_DIRECTION) ? "rx" : "tx";
		const std::string key = std::string(((dir == RX_DIRECTION) ? "RX" : "TX")) + std::string(((dspno == 0) ? "1" : "2"));
		const fs_path rf_fe_path = mb_path / "dboards" / "A" / (x + "_frontends") / (dspno ? "B" : "A");

		_tree->create<std::string>(rf_fe_path / "name").set("FE-" + key);
		// _tree->create<uhd::sensor_value_t>(rf_fe_path / "sensors/temp").set_publisher(std::bind(&iqtaxi_impl::get_temp, this));
		// _tree->create<uhd::sensor_value_t>(rf_fe_path / "sensors/lo_locked").set_publisher(std::bind(&iqtaxi_impl::get_lo_locked, this, dir, dspno));



		// if (dir == RX_DIRECTION) {
		// 	_tree->create<uhd::sensor_value_t>(rf_fe_path / "sensors/rssi").set_publisher(std::bind(&iqtaxi_impl::get_rssi, this));
		// }

		_tree->create<meta_range_t>(rf_fe_path / "gains" / "PGA" / "range").set_publisher(std::bind(&iqtaxi_impl::getGainRange, this, dir, dspno, "Normal"));
		_tree->create<double>(rf_fe_path / "gains" / "PGA" / "value")
			.set_publisher(std::bind(&iqtaxi_impl::getGain, this, dir, dspno))
			.add_coerced_subscriber(std::bind(&iqtaxi_impl::setGain, this, dir, dspno, std::placeholders::_1));


		_tree->create<std::string>(rf_fe_path / "connection").set("IQ");
		_tree->create<bool>(rf_fe_path / "enabled").set(true);
		_tree->create<bool>(rf_fe_path / "use_lo_offset").set(false);


		_tree->create<double>(rf_fe_path / "bandwidth" / "value")
			.set_publisher(std::bind(&iqtaxi_impl::getBandwidth, this, dir, dspno))
			.add_coerced_subscriber(std::bind(&iqtaxi_impl::setBandwidth, this, dir, dspno, std::placeholders::_1));
		_tree->create<uhd::meta_range_t>(rf_fe_path / "bandwidth" / "range")
			.set_publisher(std::bind(&iqtaxi_impl::getBandwidthRange, this, dir, dspno));

		_tree->create<double>(rf_fe_path / "freq" / "value")
			.set_publisher(std::bind(&iqtaxi_impl::getFrequency, this, dir, dspno, "RF"))
			.add_coerced_subscriber(std::bind(&iqtaxi_impl::set_frequency, this, dir, "RF", std::placeholders::_1));
		_tree->create<uhd::meta_range_t>(rf_fe_path / "freq" / "range")
			.set_publisher(std::bind(&iqtaxi_impl::getFrequencyRange, this, dir, dspno, "RF"));


		if (dir == RX_DIRECTION) {

			_tree->create<bool>(rf_fe_path / "dc_offset/enable")
				.set_publisher(std::bind(&iqtaxi_impl::getDCOffsetMode, this, dir, dspno))
				.add_coerced_subscriber(std::bind(&iqtaxi_impl::setDCOffsetMode, this, dir, dspno, std::placeholders::_1));

			_tree->create<bool>(rf_fe_path / "iq_balance/enable").set(true);

			const std::list<std::string> mode_strings = boost::assign::list_of("slow")("fast");
			_tree->create<bool>(rf_fe_path / "gain/agc/enable").set(false);

			_tree->create<std::string>(rf_fe_path / "gain/agc/mode/value").set(mode_strings.front());
			_tree->create< std::list<std::string> >(rf_fe_path / "gain/agc/mode/options").set(mode_strings);
		}

			const std::string antenna = dir == RX_DIRECTION ? "RX" : "TX";
			const std::vector<std::string> antennas(1, antenna);
			_tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options")
				.set(antennas);
			_tree->create<std::string>(rf_fe_path / "antenna" / "value")
				.set_coercer(std::bind(&check_e100_option_valid,
					antenna + " antenna", antennas, std::placeholders::_1))
				.add_coerced_subscriber(std::bind(&iqtaxi_impl::setAntenna,
					this, dir, dspno, std::placeholders::_1))
				.set(antenna);
	}
}

/* mirophasse
 * This function is just send 8 bytes to
 * let the fpga know the which port
 * it should send
 * */
void iqtaxi_impl::
_program_dispatcher(zero_copy_if &xport)
{
    sdr::core::managed_send_buffer::sptr buff = xport.get_send_buff();
    buff->cast<uint32_t *>()[0] = 0;
    buff->cast<uint32_t *>()[1] = uhd::htonx<uint32_t>(str_to_uint64(MICROPHASE_RX_WAZZUP_BR0));
    buff->commit(8);
    buff.reset();
}


void iqtaxi_impl::set_mb_eeprom()
{
    /* we need do no things */

}



uhd::usrp::subdev_spec_t  iqtaxi_impl::get_frontend_mapping(const uhd::direction_t dir) {

	uhd::usrp::subdev_spec_t spec;

	spec.push_back(subdev_spec_pair_t("A", "A"));
	// spec.push_back(subdev_spec_pair_t("A", "B"));

	return spec;
}


void iqtaxi_impl::set_frontend_mapping(const uhd::direction_t dir, const uhd::usrp::subdev_spec_t &spec) {

	if (spec.size() > 2) {
		std::cout << "E100 only 2 channels";
		std::runtime_error("E100 only 2 channels");
	}

	if (dir == RX_DIRECTION) {
		for (size_t i = 0; i < spec.size(); i++) {
			_rx_frontend_map[i] = (spec[i].sd_name == "A") ? 0 : 1;
			// for (size_t i = 0; i < _rx_frontend_map.size(); i++) {
			// 	LMS7002M::PathRFE path = (_rx_frontend_map[i] == 0) ? LMS7002M::PATH_RFE_LNAL : LMS7002M::PATH_RFE_LNAH;
			// 	auto rfic = getRFIC(i);
			// 	rfic->SetPathRFE(path);
			// }
		}
	}

	if (dir == TX_DIRECTION) {

		for (size_t i = 0; i < spec.size(); i++) {
			_tx_frontend_map[i] = (spec[i].sd_name == "A") ? 0 : 1;
		}

		// for (size_t i = 0; i < _tx_frontend_map.size(); i++) {
		// 	for (size_t i = 0; i < _tx_frontend_map.size(); i++) {
		// 		LMS7002M::PathRFE path = (_tx_frontend_map[i] == 0) ? LMS7002M::PATH_RFE_LB1 : LMS7002M::PATH_RFE_LB2;
		// 		auto rfic = getRFIC(i);
		// 		rfic->SetBandTRF(path);
		// 	}
		// }
	}
}


uhd::meta_range_t iqtaxi_impl::get_sample_range(){
    if (_is_m300) {
        return uhd::meta_range_t(2.083333e6, 61.44e6);
    }
    return uhd::meta_range_t(1.92e6, 122.88e6);
}

double iqtaxi_impl::getSampleRate(const uhd::direction_t dir, const size_t channel){

    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    return (double)iqtaxi_device->getSampleRate();
}

void iqtaxi_impl::setSampleRate(const uhd::direction_t dir, const size_t channel, const double rate){
    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    // E200 RX and TX share the AD9361/FPGA sample clock. UHD sets the two
    // direction properties separately and also refreshes them when a streamer
    // is created. Reprogramming an already-selected rate takes long enough to
    // create a large LTE TTI jump and can disturb an active RX stream.
    set_sample_clock_preserving_time(rate);
    // UHD clients can create their streamers before selecting the final LTE
    // sample rate.  Keep their conversion between time_spec and FPGA sample
    // ticks synchronized with the rate that was actually programmed.
    update_tick_rate(_tick_rate);
}

double iqtaxi_impl::getGain(const uhd::direction_t dir, const size_t channel){
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    if (dir== TX_DIRECTION){
			// IQTAXI and the E200 firmware expose AD9361 attenuation, while
			// UHD exposes TX gain.  Keep the low-level API in attenuation dB
			// and invert only at the UHD boundary so a larger UHD/srsRAN value
			// always means a stronger transmitted signal.
			const double attenuation = static_cast<double>(iqtaxi_device->get_tx_atten(channel));
			return _profile ? _profile->tx_attenuation_db.maximum - attenuation : attenuation;
		} else
	{
		return (double)iqtaxi_device->get_rx_gain(channel);
	}
}

void iqtaxi_impl::setGain(const uhd::direction_t dir, const size_t channel, const double gain){
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    if (dir== TX_DIRECTION){
			if (!_profile) {
				iqtaxi_device->set_tx_atten(static_cast<uint32_t>(std::llround(gain)), channel);
				return;
			}

			const auto& range = _profile->tx_attenuation_db;
			const double max_gain = range.maximum - range.minimum;
			const double step = range.step > 0.0 ? range.step : 1.0;
			const double clipped_gain = std::clamp(gain, 0.0, max_gain);
			const double quantized_gain = std::round(clipped_gain / step) * step;
			const double attenuation = range.maximum - quantized_gain;
			iqtaxi_device->set_tx_atten(static_cast<uint32_t>(std::llround(attenuation)), channel);
		} else
	{
		iqtaxi_device->set_rx_gain((uint32_t)gain,channel);
	}
}
uhd::meta_range_t iqtaxi_impl::getGainRange(const uhd::direction_t dir, const size_t chan, const std::string &name){
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    if (_profile) {
        if (dir == TX_DIRECTION) {
			return uhd::meta_range_t(0.0,
									 _profile->tx_attenuation_db.maximum -
									     _profile->tx_attenuation_db.minimum,
									 _profile->tx_attenuation_db.step);
        }
        return uhd::meta_range_t(_profile->rx_gain_db.minimum,
                                 _profile->rx_gain_db.maximum,
                                 _profile->rx_gain_db.step);
    }

    return uhd::meta_range_t(0.0, 0.0, 1.0);
}

double iqtaxi_impl::getFrequency(const uhd::direction_t dir, const size_t channel, const std::string &name){
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    if (name == "RF")
        {
        if (dir == RX_DIRECTION) {
            return (double) iqtaxi_device->get_rx_freq(channel);
        } else {
            return (double) iqtaxi_device->get_tx_freq(channel);
        }
    } else {
        return 0.0;
    }
}
void iqtaxi_impl::set_frequency(const uhd::direction_t dir, const std::string &name, const double frequency){
    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    uint64_t freq;
    freq = (uint64_t) frequency;
    if (name == "RF")
	{
        if (dir == RX_DIRECTION) {
            iqtaxi_device->set_rx_freq((uint64_t)frequency,0);
        } else {
            iqtaxi_device->set_tx_freq((uint64_t)frequency,0);
        }
    }
}


uhd::meta_range_t iqtaxi_impl::getFrequencyRange(const uhd::direction_t dir, const size_t chan, const std::string &name){
    if (name == "RF")
	{
        if (_profile) {
            const NumericRange& range =
                (dir == TX_DIRECTION) ? _profile->tx_frequency_hz : _profile->rx_frequency_hz;
            return uhd::meta_range_t(range.minimum, range.maximum, range.step);
        }
        return uhd::meta_range_t(70e6, 6e9, 1.0);
    } else {
        if (dir == RX_DIRECTION) {
            return uhd::meta_range_t(-_tick_rate/2, _tick_rate/2);
        } else {
            return uhd::meta_range_t(-_tick_rate/2, _tick_rate/2);
        }
    }

}

void iqtaxi_impl::old_issue_stream_cmd(const size_t chan, const uhd::stream_cmd_t &cmd){
    uhd::rx_streamer::sptr stream = _rx_streamer.lock();
	if (stream){
        std::cout << "issue stream command" << std::endl;
        stream->issue_stream_cmd(cmd);
    }
        
}

void iqtaxi_impl::setAntenna(const uhd::direction_t direction,
                             const size_t,
                             const std::string& name)
{
    const std::string expected = direction == RX_DIRECTION ? "RX" : "TX";
    if (name != expected) {
        throw uhd::value_error(
            str(boost::format("Invalid %s antenna: %s") % expected % name));
    }
}

double iqtaxi_impl::getBandwidth(const uhd::direction_t dir, const size_t channel){
    if (_is_m300) {
        auto m300 = std::dynamic_pointer_cast<M300XdmaImpl>(iqtaxi_device);
        return m300 ? static_cast<double>(m300->get_bandwidth()) : 0.0;
    }
    return 10e6;
}
void iqtaxi_impl::setBandwidth(const uhd::direction_t dir, const size_t channel, const double bw){
    if (_is_m300) {
        auto m300 = std::dynamic_pointer_cast<M300XdmaImpl>(iqtaxi_device);
        if (m300) m300->set_bandwidth(bw <= 0.0 ? 0u : static_cast<uint32_t>(bw));
    }
}
uhd::meta_range_t iqtaxi_impl::getBandwidthRange(const uhd::direction_t dir, const size_t channel){
    if (_is_m300) {
        return uhd::meta_range_t(200e3, 56e6, 1.0);
    }
    return uhd::meta_range_t(200e3, 100e6, 100e3);
}

void iqtaxi_impl::setDCOffsetMode(const uhd::direction_t direction, const size_t channel, const bool automatic){

}

void iqtaxi_impl::setDCOffset(const uhd::direction_t direction, const size_t channel, const std::complex<double> &offset){

}

void iqtaxi_impl::setIQBalance(const uhd::direction_t direction, const size_t channel, const std::complex<double> &balance){

}

bool iqtaxi_impl::getDCOffsetMode(const uhd::direction_t direction, const size_t channel){
    return false;
}

void iqtaxi_impl::setAutoTickRate(const bool enable){

}

sensor_value_t iqtaxi_impl::get_ref_locked(void)
{
    if (_clock_source == "internal") {
        return sensor_value_t("Ref", true, "locked", "unlocked");
    }
    if (_is_m300 || !_local_bus) {
        return sensor_value_t("Ref", false, "locked", "unlocked");
    }

    // E200 packs the VCXO loop status as:
    // [0] locked, [1] reference valid, [2] 10 MHz detected,
    // [3] PPS detected, [5:4] selected source, [31:16] DAC value.
    // An external UHD reference is usable only when the 10 MHz input is
    // present, selected, and the VCXO disciplining loop has acquired lock.
    const uint32_t status = _local_bus->peek32(CUSTOM_RB_GET_VCXO_STATUS, 1.0);
    const bool lock = (status & 0x7u) == 0x7u && ((status >> 4) & 0x3u) == 1u;
    return sensor_value_t("Ref", lock, "locked", "unlocked");
}

void iqtaxi_impl::set_time(const uhd::time_spec_t& t)
{
    set_time_sync(t);
    if (_is_m300 || !_local_bus) return;
    _local_bus->poke32(CUSTOM_SET_TIME_SYNC, 1 << 2 | uint32_t(_time_source));
    _local_bus->poke32(CUSTOM_SET_TIME_SYNC, _time_source);
}

void iqtaxi_impl::sync_times()
{
    set_time(get_time_now());
}



uhd::time_spec_t iqtaxi_impl::get_time_now(void)
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    const uint64_t ticks = iqtaxi_device->getTimeTicks();
    return uhd::time_spec_t::from_ticks(ticks, _tick_rate);
}

uhd::time_spec_t iqtaxi_impl::get_time_last_pps(void)
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    if (_is_m300 || !_local_bus) {
        const uint64_t ticks = iqtaxi_device->getTimeTicks();
        return uhd::time_spec_t::from_ticks(ticks, _tick_rate);
    }

    const uint64_t ticks = _local_bus->peek64(CUSTOM_RB_GET_VITA_TIME_LAST_PPS_ADDR);
    return uhd::time_spec_t::from_ticks(ticks, _tick_rate);
}

void iqtaxi_impl::set_time_now(const uhd::time_spec_t& time)
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    const uint64_t ticks = time.to_ticks(_tick_rate);
    iqtaxi_device->setTimestamp(ticks,E100_CTRL_LATCH_TIME_NOW);
}

void iqtaxi_impl::set_time_sync(const uhd::time_spec_t& time)
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    const uint64_t ticks = time.to_ticks(_tick_rate);
    iqtaxi_device->setTimestamp(ticks,E100_CTRL_LATCH_TIME_SYNC);
}

void iqtaxi_impl::set_time_next_pps(const uhd::time_spec_t& time)
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    
    const uint64_t ticks = time.to_ticks(_tick_rate);
    iqtaxi_device->setTimestamp(ticks, E100_CTRL_LATCH_TIME_PPS);
}

void iqtaxi_impl::update_time_source(const std::string& source)
{
    // if ((_product == B200MINI or _product == B205MINI) and source == "external"
    //     and _gpio_state.ref_sel == 1) {
    //     throw uhd::value_error(
    //         "external reference cannot be both a time source and a clock source");
    // }

    // // We assume source is valid for this device (if it's gone through
    // // the prop three, then it definitely is thanks to our coercer)
    time_source_t value;
    if (source == "none")
        value = NONE;
    else if (source == "internal")
        value = INTERNAL;
    else if (source == "external")
        value = EXTERNAL;
    else
        throw uhd::key_error("update_time_source: unknown source: " + source);
    if (_time_source != value && !_is_m300 && _local_bus) {
        _local_bus->poke32(CUSTOM_SET_TIME_SYNC, value);
    }
    _time_source = value;
}


void iqtaxi_impl::update_clock_source(const std::string& source)
{
    if (_is_m300 || !_local_bus) {
        if (source != "internal") {
            throw uhd::value_error("external clock source is not supported by this backend");
        }
        _clock_source = source;
        return;
    }

    if (source == "external") {
        const uint64_t response = _local_bus->poke32_ack_value(
            CUSTOM_SET_VCXO_REF_SOURCE,
            static_cast<uint32_t>(E200Impl::VcxoReferenceSource::external_10mhz),
            1.0);
        if (response != 0u) {
            throw uhd::runtime_error("failed to select the external 10 MHz VCXO reference");
        }
    } else if (source == "internal") {
        // Freeze the last loop DAC value before switching to free-running
        // operation. This avoids a frequency step when returning from an
        // external reference and gives UHD's "internal" source its expected
        // local-oscillator semantics.
        const uint32_t status = _local_bus->peek32(CUSTOM_RB_GET_VCXO_STATUS, 1.0);
        const uint32_t dac_value = status >> 16;
        uint64_t response = _local_bus->poke32_ack_value(
            CUSTOM_SET_VCXO_DAC_VALUE, dac_value, 1.0);
        if (response == 0u) {
            response = _local_bus->poke32_ack_value(
                CUSTOM_SET_VCXO_REF_SOURCE,
                static_cast<uint32_t>(E200Impl::VcxoReferenceSource::manual_dac),
                1.0);
        }
        if (response != 0u) {
            throw uhd::runtime_error("failed to select the internal VCXO reference");
        }
    } else {
        throw uhd::key_error("update_clock_source: unknown source: " + source);
    }

    _clock_source = source;
}

double iqtaxi_impl::set_tick_rate(const double new_tick_rate)
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);

    UHD_LOGGER_INFO("ANT") << (boost::format("Asking for clock rate %.6f MHz... ")
                                % (new_tick_rate / 1e6))
                            << std::flush;
    // check_tick_rate_with_current_streamers(new_tick_rate); // Defined in b200_io_impl.cpp

    // Make sure the clock rate is actually changed before doing
    // the full Monty of setting regs and loopback tests etc.
    // if (std::abs(new_tick_rate - _tick_rate) < 1.0) {
    //     UHD_LOGGER_INFO("ANT") << "OK";
    //     return _tick_rate;
    // }
    set_sample_clock_preserving_time(new_tick_rate);
    UHD_LOGGER_INFO("ANT") << (boost::format("Actually got clock rate %.6f MHz.")
                                % (_tick_rate / 1e6));

    return _tick_rate;
}

double iqtaxi_impl::set_sample_clock_preserving_time(const double requested_rate)
{
    const uint32_t target_rate = static_cast<uint32_t>(requested_rate);
    if (target_rate == 0u) {
        throw uhd::value_error("sample clock rate must be non-zero");
    }

    /* The UHD property tree can update _tick_rate before the radio rate
     * subscriber runs.  In that case _tick_rate alone is not proof that the
     * board was programmed: after a previous client leaves the E206 at a
     * different rate, the old shortcut silently kept that hardware rate.
     * Always compare against the control-plane readback as well. */
    const double current_device_rate = iqtaxi_device->getSampleRate();
    if (std::abs(_tick_rate - static_cast<double>(target_rate)) < 1.0 &&
        std::abs(current_device_rate - static_cast<double>(target_rate)) < 1.0) {
        return _tick_rate;
    }

    // The E-series FPGA timekeeper advances once per AD9361 sample.  Its raw
    // counter therefore changes units when the sample clock changes.  UHD
    // time_spec, however, is absolute time and must remain monotonic across a
    // rate change (as it does on a USRP).  Capture the old time in seconds and
    // rebase the counter after the lengthy AD9361 clock/FIR reconfiguration.
    // The stream is stopped while this happens, so SDR sample time must remain
    // frozen rather than including wall-clock reconfiguration latency.  This
    // avoids both a backwards jump and an artificial multi-thousand-TTI gap.
    if (_is_m300) {
        iqtaxi_device->setSampleRate(target_rate);
        _tick_rate = iqtaxi_device->getSampleRate();
        return _tick_rate;
    }

    const double old_rate = current_device_rate;
    const uint64_t old_ticks = iqtaxi_device->getTimeTicks();
    const uhd::time_spec_t old_time =
        uhd::time_spec_t::from_ticks(old_ticks, old_rate);
    iqtaxi_device->setSampleRate(target_rate);
    const double actual_rate = iqtaxi_device->getSampleRate();
    iqtaxi_device->setTimestamp(
        old_time.to_ticks(actual_rate), E100_CTRL_LATCH_TIME_NOW);
    _tick_rate = actual_rate;
    return _tick_rate;
}
     
/***********************************************************************
 * frontend selection
 **********************************************************************/
uhd::usrp::subdev_spec_t iqtaxi_impl::coerce_subdev_spec(
    const uhd::usrp::subdev_spec_t& spec_)
{
    uhd::usrp::subdev_spec_t spec = spec_;
    // Because of the confusing nature of the subdevs on ANT
    // with different revs, we provide a convenience override,
    // where both A:A and A:B are mapped to A:A.
    //
    // Any other spec is probably illegal and will be caught by
    // validate_subdev_spec().
    // if (!spec.empty()
    //     and (_product == B200 or _product == B200MINI or _product == B205MINI)
    //     and spec[0].sd_name == "B") {
    //     spec[0].sd_name = "A";
    // }

    if (!spec.empty() and spec[0].sd_name == "B") {
        spec[0].sd_name = "A";
    }
    // spec[0].sd_name = "A";
    return spec;
}

void iqtaxi_impl::update_tick_rate(const double new_tick_rate)
{
    for (radio_perifs_t& perif : _radio_perifs) {
        if (auto stream = std::dynamic_pointer_cast<iqtaxi_rx_streamer>(
                perif.rx_streamer.lock())) {
            stream->set_tick_rate(new_tick_rate);
        }
        if (auto stream = std::dynamic_pointer_cast<iqtaxi_tx_streamer>(
                perif.tx_streamer.lock())) {
            stream->set_tick_rate(new_tick_rate);
        }
    }
}

void iqtaxi_impl::update_bandsel(const std::string& which, double freq)
{

}
