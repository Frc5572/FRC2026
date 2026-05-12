#include "httplib.hh"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

std::string read_file(const std::string &path)
{
    std::ifstream file(path);

    if (!file.is_open())
    {
        return "Failed to open file.";
    }

    std::stringstream buffer;

    buffer << file.rdbuf();

    return buffer.str();
}

int main()
{
    httplib::Server server;

    // Serve HTML file
    server.Get("/", [](const httplib::Request &,
                       httplib::Response &res)
               {
        std::string html = read_file("/Users/jacerodgers/Code/FRC2026/coprocessor/mac/gtsam_D/src/index.html");

        res.set_content(html, "text/html"); });

    // Upload endpoint
    server.Post("/upload", [](const httplib::Request &req,
                              httplib::Response &res)
                {
        std::ofstream out("uploaded.json");

        out << req.body;

        out.close();

        std::cout << req.body << std::endl;

        res.set_content(
            "JSON uploaded successfully.",
            "text/plain"
        ); });

    std::cout << "Server running at http://localhost:8080\n";

    server.listen("0.0.0.0", 8080);
}