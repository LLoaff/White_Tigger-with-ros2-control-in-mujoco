#include "math/data_analyze.h"

data_analyze::data_analyze(/* args */)
{
}

void data_analyze::sendComPos(double sim_time, const Eigen::Vector3d& pcom) {
    CURL* curl = curl_easy_init();
    if (!curl) return;

    std::ostringstream ss;
    ss << "{"
       << "\"sim_time\":" << sim_time << ","
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
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 5L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    curl_easy_perform(curl);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
}

data_analyze::~data_analyze()
{
}