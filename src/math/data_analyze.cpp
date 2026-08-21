#include "math/data_analyze.h"

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

using json = nlohmann::json;

namespace
{
size_t writeCallback(char* data, size_t size, size_t count, void* output)
{
    const size_t dataSize = size * count;
    static_cast<std::string*>(output)->append(data, dataSize);
    return dataSize;
}

bool requestDebugParams(const std::string* postBody, std::string& response)
{
    CURL* curl = curl_easy_init();
    if (!curl) return false;

    curl_slist* headers = nullptr;
    curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8765/api/debug-params");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 100L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    if (postBody) {
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postBody->c_str());
    }

    const CURLcode result = curl_easy_perform(curl);
    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return result == CURLE_OK && httpCode >= 200 && httpCode < 300;
}

json diagonalToJson(const Mat3& matrix)
{
    return {matrix(0, 0), matrix(1, 1), matrix(2, 2)};
}

Mat3 jsonToDiagonal(const json& value)
{
    return Vec3(value.at(0).get<double>(),
                value.at(1).get<double>(),
                value.at(2).get<double>()).asDiagonal();
}
}

data_analyze::data_analyze(): _lastSendTime(-1.0)
{
}

void data_analyze::sendComPos(double sim_time, const Eigen::Vector3d& pcom, const Eigen::Vector3d& rpy) {
    constexpr double sendInterval = 0.02;  // 50 Hz telemetry
    if (_lastSendTime >= 0.0 && sim_time >= _lastSendTime && sim_time - _lastSendTime < sendInterval) {
        return;
    }
    _lastSendTime = sim_time;

    CURL* curl = curl_easy_init();
    if (!curl) return;

    std::ostringstream ss;
    ss << "{"
       << "\"sim_time\":" << sim_time << ","
       << "\"imu\":{\"rpy\":["
       << rpy(0) << "," << rpy(1) << "," << rpy(2)
       << "]},"
       << "\"com\":{\"pos\":["
       << pcom(0) << "," << pcom(1) << "," << pcom(2)
       << "]}"
       << "}";

    std::string body = ss.str();

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8765/api/telemetry");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    std::string response;
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 5L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    curl_easy_perform(curl);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
}

bool data_analyze::sendParamData(const param& data)
{
    json body;
    json& trotting = body["params"]["trotting"];
    trotting["Kpp"] = diagonalToJson(data._Kpp);
    trotting["Kdp"] = diagonalToJson(data._Kdp);
    trotting["kpw"] = data._kpw;
    trotting["Kdw"] = diagonalToJson(data._Kdw);
    trotting["KpSwing"] = diagonalToJson(data._KpSwing);
    trotting["KdSwing"] = diagonalToJson(data._KdSwing);

    const std::string postBody = body.dump();
    std::string response;
    return requestDebugParams(&postBody, response);
}

bool data_analyze::getParamData(param& data)
{
    std::string response;
    if (!requestDebugParams(nullptr, response)) return false;

    try {
        const json root = json::parse(response);
        const json& trotting = root.at("params").at("trotting");
        param next;
        next._Kpp = jsonToDiagonal(trotting.at("Kpp"));
        next._Kdp = jsonToDiagonal(trotting.at("Kdp"));
        next._kpw = trotting.at("kpw").get<double>();
        next._Kdw = jsonToDiagonal(trotting.at("Kdw"));
        next._KpSwing = jsonToDiagonal(trotting.at("KpSwing"));
        next._KdSwing = jsonToDiagonal(trotting.at("KdSwing"));
        data = next;
        return true;
    } catch (const json::exception&) {
        return false;
    }
}
