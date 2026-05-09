#define CPPHTTPLIB_THREAD_POOL_COUNT 1

#include "httplib.hh"

int main()
{
    httplib::Server svr;

    svr.Get("/", [](const httplib::Request &, httplib::Response &res)
            { res.set_content("hello", "text/plain"); });

    svr.listen("0.0.0.0", 8080);
}