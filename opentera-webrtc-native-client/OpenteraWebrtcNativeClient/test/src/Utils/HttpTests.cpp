#include <OpenteraWebrtcNativeClient/Utils/Http.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace introlab;
using namespace std;

using ::testing::HasSubstr;

TEST(HttpTests, get_http_shouldReturnTrueAndSetResponse)
{
    auto requestModifier = [](boost::beast::http::request<boost::beast::http::string_body>&) {};
    string response;
    EXPECT_TRUE(Http::get("http://www.perdus.com/", response, requestModifier));
    EXPECT_THAT(response, HasSubstr("Vous Etes Perdus ?"));
}

TEST(HttpTests, get_https_shouldReturnTrueAndSetResponse)
{
    auto requestModifier = [](boost::beast::http::request<boost::beast::http::string_body>&) {};
    string response;
    EXPECT_TRUE(Http::get("https://www.perdus.com/", response, requestModifier));
    EXPECT_THAT(response, HasSubstr("Vous Etes Perdus ?"));
}

TEST(HttpTests, getHostPortTargetFromUrl_invalidUrl_shouldReturnFalse)
{
    string host;
    string port;
    string target;

    EXPECT_FALSE(Http::getHostPortTargetFromUrl("", "80", host, port, target));
    EXPECT_FALSE(Http::getHostPortTargetFromUrl("asd", "80", host, port, target));
}

TEST(HttpTests, getHostPortTargetFromUrl_validUrl_shouldReturnTrueAndSetParameters)
{
    string host;
    string port;
    string target;

    EXPECT_TRUE(Http::getHostPortTargetFromUrl("http://localhost", "80", host, port, target));
    EXPECT_EQ(host, "localhost");
    EXPECT_EQ(port, "80");
    EXPECT_EQ(target, "/");

    EXPECT_TRUE(Http::getHostPortTargetFromUrl("http://localhost/a/route/", "80", host, port, target));
    EXPECT_EQ(host, "localhost");
    EXPECT_EQ(port, "80");
    EXPECT_EQ(target, "/a/route/");

    EXPECT_TRUE(Http::getHostPortTargetFromUrl("http://localhost:8080", "80", host, port, target));
    EXPECT_EQ(host, "localhost");
    EXPECT_EQ(port, "8080");
    EXPECT_EQ(target, "/");

    EXPECT_TRUE(Http::getHostPortTargetFromUrl("http://localhost:6060/a/route/", "80", host, port, target));
    EXPECT_EQ(host, "localhost");
    EXPECT_EQ(port, "6060");
    EXPECT_EQ(target, "/a/route/");
}
