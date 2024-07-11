#include <OpenteraWebrtcNativeClient/Utils/Http.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

using ::testing::HasSubstr;

TEST(HttpTests, get_http_shouldReturnTrueAndSetResponse)
{
    string response;
    if (Http::get("http://www.perdu.com", response, {}))
    {
        EXPECT_THAT(response, HasSubstr("Vous Etes Perdu ?"));
    }
    else if (Http::get("http://www.perdus.com", response, {}))
    {
        EXPECT_THAT(response, HasSubstr("Vous Etes Perdus ?"));
    }
    else
    {
        FAIL() << "Neither 'http://www.perdus.com' nor 'http://www.perdu.com' could be reached.";
    }
}

TEST(HttpTests, get_https_shouldReturnTrueAndSetResponse)
{
    string response;
    if (Http::get("https://www.perdu.com", response, {}))
    {
        EXPECT_THAT(response, HasSubstr("Vous Etes Perdu ?"));
    }
    else if (Http::get("https://www.perdus.com", response, {}))
    {
        EXPECT_THAT(response, HasSubstr("Vous Etes Perdus ?"));
    }
    else
    {
        FAIL() << "Neither 'https://www.perdu.com' nor 'https://www.perdus.com' could be reached.";
    }
}

TEST(HttpTests, get_invalidUrl_shouldReturnFalse)
{
    string response;
    EXPECT_FALSE(Http::get("http://localhost:8080/iceservers", response, {}));
    EXPECT_EQ(response, "");

    EXPECT_FALSE(Http::get("https://localhost:8080/iceservers", response, {}));
    EXPECT_EQ(response, "");
}

TEST(HttpTests, splitUrl_invalidUrl_shouldReturnFalse)
{
    string host;
    string target;

    EXPECT_FALSE(Http::splitUrl("", host, target));
    EXPECT_FALSE(Http::splitUrl("asd", host, target));
}

TEST(HttpTests, getHostPortTargetFromUrl_validUrl_shouldReturnTrueAndSetParameters)
{
    string host;
    string target;

    EXPECT_TRUE(Http::splitUrl("http://localhost", host, target));
    EXPECT_EQ(host, "http://localhost");
    EXPECT_EQ(target, "/");

    EXPECT_TRUE(Http::splitUrl("http://localhost/a/route/", host, target));
    EXPECT_EQ(host, "http://localhost");
    EXPECT_EQ(target, "/a/route/");

    EXPECT_TRUE(Http::splitUrl("http://localhost:8080", host, target));
    EXPECT_EQ(host, "http://localhost:8080");
    EXPECT_EQ(target, "/");

    EXPECT_TRUE(Http::splitUrl("http://localhost:6060/a/route/", host, target));
    EXPECT_EQ(host, "http://localhost:6060");
    EXPECT_EQ(target, "/a/route/");
}
