#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(ClientTests, constructor_Client_shouldSetTheAttributes)
{
    Client testee1("id1", "name1", sio::string_message::create("data1"));

    EXPECT_EQ(testee1.id(), "id1");
    EXPECT_EQ(testee1.name(), "name1");
    ASSERT_EQ(testee1.data()->get_flag(), sio::message::flag_string);
    EXPECT_EQ(testee1.data()->get_string(), "data1");

    auto message = sio::object_message::create();
    message->get_map()["id"] = sio::string_message::create("id2");
    message->get_map()["name"] = sio::string_message::create("name2");
    message->get_map()["data"] = sio::string_message::create("data2");
    Client testee2(message);

    EXPECT_EQ(testee2.id(), "id2");
    EXPECT_EQ(testee2.name(), "name2");
    ASSERT_EQ(testee2.data()->get_flag(), sio::message::flag_string);
    EXPECT_EQ(testee2.data()->get_string(), "data2");
}

TEST(ClientTests, isValid_shouldReturnTrueOnlyIfTheMessageIsValid)
{
    EXPECT_FALSE(Client::isValid(sio::string_message::create("data1")));

    auto message = sio::object_message::create();
    EXPECT_FALSE(Client::isValid(message));

    message->get_map()["id"] = sio::null_message::create();
    EXPECT_FALSE(Client::isValid(message));

    message->get_map()["name"] = sio::null_message::create();
    EXPECT_FALSE(Client::isValid(message));

    message->get_map()["data"] = sio::null_message::create();
    EXPECT_FALSE(Client::isValid(message));

    message->get_map()["id"] = sio::string_message::create("id");
    EXPECT_FALSE(Client::isValid(message));

    message->get_map()["name"] = sio::string_message::create("name");
    EXPECT_TRUE(Client::isValid(message));
}

TEST(ClientTests, equalityOperator_Client_shouldReturnFalseIfOperandsAreNotEqual)
{
    EXPECT_FALSE(Client("c1", "n1", sio::string_message::create("d")) ==
        Client("c2", "n1", sio::string_message::create("d")));
    EXPECT_FALSE(Client("c1", "n1", sio::string_message::create("d")) ==
        Client("c1", "n2", sio::string_message::create("d")));
    EXPECT_FALSE(Client("c1", "n1", sio::string_message::create("d")) ==
        Client("c1", "n1", sio::string_message::create("e")));
    EXPECT_FALSE(Client("c1", "n1", sio::string_message::create("d")) ==
        Client("c1", "n1", sio::int_message::create(10)));
    EXPECT_FALSE(Client("c1", "n1", sio::string_message::create("d")) ==
        Client("c1", "n1", nullptr));
    EXPECT_FALSE(Client("c1", "n1", nullptr) ==
        Client("c1", "n1", sio::int_message::create(10)));

    auto array1 = sio::array_message::create();
    array1->get_vector().emplace_back(sio::string_message::create("a"));
    auto array2 = sio::array_message::create();
    EXPECT_FALSE(Client("c1", "n1", array1) == Client("c1", "n1", array2));
    array2->get_vector().emplace_back(sio::string_message::create("b"));
    EXPECT_FALSE(Client("c1", "n1", array1) == Client("c1", "n1", array2));

    auto object1 = sio::object_message::create();
    object1->get_map()["x"] = sio::null_message::create();
    object1->get_map()["bob"] = sio::string_message::create("d");
    auto object2 = sio::object_message::create();
    object2->get_map()["x"] = sio::string_message::create("d");
    EXPECT_FALSE(Client("c1", "n1", object1) == Client("c1", "n1", object2));
}

TEST(ClientTests, equalityOperator_Client_shouldReturnTrueIfOperandsAreEqual)
{
    EXPECT_TRUE(Client("c", "n", sio::int_message::create(10)) ==
        Client("c", "n", sio::int_message::create(10)));
    EXPECT_TRUE(Client("c", "n", sio::double_message::create(0.0)) ==
        Client("c", "n", sio::double_message::create(0.0)));
    EXPECT_TRUE(Client("c", "n", sio::binary_message::create(nullptr)) ==
        Client("c", "n", sio::binary_message::create(nullptr)));
    EXPECT_TRUE(Client("c", "n", sio::bool_message::create(false)) ==
        Client("c", "n", sio::bool_message::create(false)));
    EXPECT_TRUE(Client("c", "n", sio::null_message::create()) ==
        Client("c", "n", sio::null_message::create()));
    EXPECT_TRUE(Client("c", "n", nullptr) ==
        Client("c", "n", nullptr));

    auto array1 = sio::array_message::create();
    array1->get_vector().emplace_back(sio::string_message::create("a"));
    auto array2 = sio::array_message::create();
    array2->get_vector().emplace_back(sio::string_message::create("a"));
    EXPECT_TRUE(Client("c1", "n1", array1) == Client("c1", "n1", array2));

    auto object1 = sio::object_message::create();
    object1->get_map()["x"] = sio::null_message::create();
    object1->get_map()["bob"] = sio::string_message::create("d");
    auto object2 = sio::object_message::create();
    object2->get_map()["x"] = sio::null_message::create();
    object2->get_map()["bob"] = sio::string_message::create("d");
    EXPECT_TRUE(Client("c1", "n1", object1) == Client("c1", "n1", object2));
}

TEST(ClientTests, constructor_RoomClient_shouldSetTheAttributes)
{
    RoomClient testee1("id1", "name1", sio::string_message::create("data1"), false);
    EXPECT_EQ(testee1.id(), "id1");
    EXPECT_EQ(testee1.name(), "name1");
    EXPECT_EQ(testee1.data()->get_flag(), sio::message::flag_string);
    EXPECT_EQ(testee1.data()->get_string(), "data1");
    EXPECT_EQ(testee1.isConnected(), false);

    RoomClient testee2(Client("id2", "name2", sio::string_message::create("data2")), true);
    EXPECT_EQ(testee2.id(), "id2");
    EXPECT_EQ(testee2.name(), "name2");
    EXPECT_EQ(testee2.data()->get_flag(), sio::message::flag_string);
    EXPECT_EQ(testee2.data()->get_string(), "data2");
    EXPECT_EQ(testee2.isConnected(), true);
}

TEST(ClientTests, equalityOperator_RoomClient_shouldReturnFalseIfOperandsAreNotEqual)
{
    EXPECT_FALSE(RoomClient("c1", "n1", sio::string_message::create("d"), true) ==
        RoomClient("c2", "n1", sio::string_message::create("d"), true));
    EXPECT_FALSE(RoomClient("c1", "n1", sio::string_message::create("d"), false) ==
        RoomClient("c1", "n2", sio::string_message::create("d"), false));
    EXPECT_FALSE(RoomClient("c1", "n1", sio::string_message::create("d"), false) ==
        RoomClient("c1", "n1", sio::string_message::create("e"), false));
    EXPECT_FALSE(RoomClient("c1", "n1", sio::string_message::create("d"), true) ==
        RoomClient("c1", "n1", sio::string_message::create("d"), false));
    EXPECT_FALSE(RoomClient("c1", "n1", sio::string_message::create("d"), true) ==
        RoomClient("c1", "n1", nullptr, false));
    EXPECT_FALSE(RoomClient("c1", "n1", nullptr, true) ==
        RoomClient("c1", "n1", sio::string_message::create("d"), false));

    EXPECT_FALSE(RoomClient("c1", "n1", sio::string_message::create("d"), true) == RoomClient());
    EXPECT_FALSE(RoomClient() == RoomClient("c1", "n1", sio::string_message::create("d"), true));
}

TEST(ClientTests, equalityOperator_RoomClient_shouldReturnTrueIfOperandsAreEqual)
{
    EXPECT_TRUE(RoomClient("c", "n", sio::int_message::create(10), false) ==
        RoomClient("c", "n", sio::int_message::create(10), false));
    EXPECT_TRUE(RoomClient("c", "n", nullptr, false) == RoomClient("c", "n", nullptr, false));
    EXPECT_TRUE(RoomClient() == RoomClient());
}
