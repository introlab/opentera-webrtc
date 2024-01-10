#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(ClientTests, constructor_Client_shouldSetTheAttributes)
{
    Client testee1("id1", "name1", "data1");

    EXPECT_EQ(testee1.id(), "id1");
    EXPECT_EQ(testee1.name(), "name1");
    ASSERT_TRUE(testee1.data().is_string());
    EXPECT_EQ(testee1.data(), "data1");

    nlohmann::json message{{"id", "id2"}, {"name", "name2"}, {"data", "data2"}};
    Client testee2(message);

    EXPECT_EQ(testee2.id(), "id2");
    EXPECT_EQ(testee2.name(), "name2");
    ASSERT_TRUE(testee2.data().is_string());
    EXPECT_EQ(testee2.data(), "data2");
}

TEST(ClientTests, isValid_shouldReturnTrueOnlyIfTheMessageIsValid)
{
    EXPECT_FALSE(Client::isValid("data1"));

    nlohmann::json message;
    EXPECT_FALSE(Client::isValid(message));

    message.emplace("id", nlohmann::json{});
    EXPECT_FALSE(Client::isValid(message));

    message.emplace("name", nlohmann::json{});
    EXPECT_FALSE(Client::isValid(message));

    message.emplace("data", nlohmann::json{});
    EXPECT_FALSE(Client::isValid(message));

    message["id"] = "id";
    EXPECT_FALSE(Client::isValid(message));

    message["name"] = "name";
    EXPECT_TRUE(Client::isValid(message));
}

TEST(ClientTests, equalityOperator_Client_shouldReturnFalseIfOperandsAreNotEqual)
{
    EXPECT_FALSE(Client("c1", "n1", "d") == Client("c2", "n1", "d"));
    EXPECT_FALSE(Client("c1", "n1", "d") == Client("c1", "n2", "d"));
    EXPECT_FALSE(Client("c1", "n1", "d") == Client("c1", "n1", "e"));
    EXPECT_FALSE(Client("c1", "n1", "d") == Client("c1", "n1", 10));
    EXPECT_FALSE(Client("c1", "n1", "d") == Client("c1", "n1", nullptr));
    EXPECT_FALSE(Client("c1", "n1", nullptr) == Client("c1", "n1", 10));

    auto array1 = nlohmann::json::array({"a"});
    auto array2 = nlohmann::json::array();
    EXPECT_FALSE(Client("c1", "n1", array1) == Client("c1", "n1", array2));
    array2.emplace_back("b");
    EXPECT_FALSE(Client("c1", "n1", array1) == Client("c1", "n1", array2));

    nlohmann::json object1 = {{"x", nlohmann::json{}}, {"bob", "d"}};
    nlohmann::json object2{{"x", "d"}};
    EXPECT_FALSE(Client("c1", "n1", object1) == Client("c1", "n1", object2));
}

TEST(ClientTests, equalityOperator_Client_shouldReturnTrueIfOperandsAreEqual)
{
    EXPECT_TRUE(Client("c", "n", 10) == Client("c", "n", 10));
    EXPECT_TRUE(Client("c", "n", 0.0) == Client("c", "n", 0.0));
    EXPECT_TRUE(Client("c", "n", nullptr) == Client("c", "n", nullptr));
    EXPECT_TRUE(Client("c", "n", false) == Client("c", "n", false));
    EXPECT_TRUE(Client("c", "n", nlohmann::json{}) == Client("c", "n", nlohmann::json{}));

    auto array1 = nlohmann::json::array({"a"});
    auto array2 = nlohmann::json::array({"a"});
    EXPECT_TRUE(Client("c1", "n1", array1) == Client("c1", "n1", array2));

    nlohmann::json object1 = {{"x", nlohmann::json{}}, {"bob", "d"}};
    nlohmann::json object2{{"x", nlohmann::json{}}, {"bob", "d"}};
    EXPECT_TRUE(Client("c1", "n1", object1) == Client("c1", "n1", object2));
}

TEST(ClientTests, constructor_RoomClient_shouldSetTheAttributes)
{
    RoomClient testee1("id1", "name1", "data1", false);
    EXPECT_EQ(testee1.id(), "id1");
    EXPECT_EQ(testee1.name(), "name1");
    ASSERT_TRUE(testee1.data().is_string());
    EXPECT_EQ(testee1.data(), "data1");
    EXPECT_EQ(testee1.isConnected(), false);

    RoomClient testee2(Client("id2", "name2", "data2"), true);
    EXPECT_EQ(testee2.id(), "id2");
    EXPECT_EQ(testee2.name(), "name2");
    ASSERT_TRUE(testee2.data().is_string());
    EXPECT_EQ(testee2.data(), "data2");
    EXPECT_EQ(testee2.isConnected(), true);
}

TEST(ClientTests, equalityOperator_RoomClient_shouldReturnFalseIfOperandsAreNotEqual)
{
    EXPECT_FALSE(RoomClient("c1", "n1", "d", true) == RoomClient("c2", "n1", "d", true));
    EXPECT_FALSE(RoomClient("c1", "n1", "d", false) == RoomClient("c1", "n2", "d", false));
    EXPECT_FALSE(RoomClient("c1", "n1", "d", false) == RoomClient("c1", "n1", "e", false));
    EXPECT_FALSE(RoomClient("c1", "n1", "d", true) == RoomClient("c1", "n1", "d", false));
    EXPECT_FALSE(RoomClient("c1", "n1", "d", true) == RoomClient("c1", "n1", nullptr, false));
    EXPECT_FALSE(RoomClient("c1", "n1", nullptr, true) == RoomClient("c1", "n1", "d", false));

    EXPECT_FALSE(RoomClient("c1", "n1", "d", true) == RoomClient());
    EXPECT_FALSE(RoomClient() == RoomClient("c1", "n1", "d", true));
}

TEST(ClientTests, equalityOperator_RoomClient_shouldReturnTrueIfOperandsAreEqual)
{
    EXPECT_TRUE(RoomClient("c", "n", 10, false) == RoomClient("c", "n", 10, false));
    EXPECT_TRUE(RoomClient("c", "n", nullptr, false) == RoomClient("c", "n", nullptr, false));
    EXPECT_TRUE(RoomClient() == RoomClient());
}
