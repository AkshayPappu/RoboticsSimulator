#include <gtest/gtest.h>
#include "environment/SceneLoader.h"
#include "physics/RigidBody.h"

#include <fstream>
#include <filesystem>

using namespace robosim;

class SceneLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a temp directory for test files
        testDir = std::filesystem::temp_directory_path() / "robosim_test";
        std::filesystem::create_directories(testDir);
    }

    void TearDown() override {
        std::filesystem::remove_all(testDir);
    }

    void WriteTestFile(const std::string& filename, const std::string& content) {
        auto path = testDir / filename;
        std::ofstream f(path);
        f << content;
    }

    std::string GetTestPath(const std::string& filename) {
        return (testDir / filename).string();
    }

    std::filesystem::path testDir;
};

TEST_F(SceneLoaderTest, LoadValidScene) {
    WriteTestFile("valid.json", R"({
        "name": "test_scene",
        "gravity": [0, -9.81, 0],
        "ground": { "type": "plane", "friction": 0.5, "restitution": 0.1 },
        "objects": [
            { "type": "box", "size": [1, 1, 1], "position": [0, 0.5, 0], "mass": 0, "friction": 0.5 }
        ],
        "robot_spawn": [0, 0.5, 0]
    })");

    SceneLoader loader;
    auto data = loader.Load(GetTestPath("valid.json"));

    EXPECT_EQ(data.name, "test_scene");
    EXPECT_NEAR(data.gravity[1], -9.81, 1e-6);
    EXPECT_TRUE(data.hasGround);
    EXPECT_NEAR(data.groundFriction, 0.5, 1e-6);
    EXPECT_EQ(data.objects.size(), 1);
    EXPECT_NEAR(data.robotSpawn[1], 0.5, 1e-6);
}

TEST_F(SceneLoaderTest, LoadSceneWithMultipleObjectTypes) {
    WriteTestFile("multi.json", R"({
        "name": "multi_type",
        "objects": [
            { "type": "box", "size": [1, 1, 1], "position": [0, 0.5, 0], "mass": 0 },
            { "type": "sphere", "radius": 0.5, "position": [2, 1, 0], "mass": 1.0 },
            { "type": "cylinder", "size": [0.3, 1.0], "position": [4, 0.5, 0], "mass": 0 },
            { "type": "ramp", "length": 3.0, "slope": 15, "position": [6, 0, 0] },
            { "type": "wall", "size": [0.2, 1, 5], "position": [-3, 0.5, 0], "mass": 0 }
        ]
    })");

    SceneLoader loader;
    auto data = loader.Load(GetTestPath("multi.json"));

    EXPECT_EQ(data.objects.size(), 5);
}

TEST_F(SceneLoaderTest, LoadMinimalScene) {
    WriteTestFile("minimal.json", R"({ "name": "empty" })");

    SceneLoader loader;
    auto data = loader.Load(GetTestPath("minimal.json"));

    EXPECT_EQ(data.name, "empty");
    EXPECT_EQ(data.objects.size(), 0);
}

TEST_F(SceneLoaderTest, HandleMissingFile) {
    SceneLoader loader;
    auto data = loader.Load("nonexistent_file.json");

    // Should return default scene without crashing
    EXPECT_EQ(data.name, "default");
}

TEST_F(SceneLoaderTest, HandleInvalidJson) {
    WriteTestFile("bad.json", "{ this is not valid json }}}");

    SceneLoader loader;
    auto data = loader.Load(GetTestPath("bad.json"));

    // Should return default scene without crashing
    EXPECT_EQ(data.name, "default");
}

TEST_F(SceneLoaderTest, CustomGravity) {
    WriteTestFile("moon.json", R"({
        "name": "moon",
        "gravity": [0, -1.625, 0]
    })");

    SceneLoader loader;
    auto data = loader.Load(GetTestPath("moon.json"));

    EXPECT_NEAR(data.gravity[1], -1.625, 1e-6);
}

TEST_F(SceneLoaderTest, StaticAndDynamicObjects) {
    WriteTestFile("mixed.json", R"({
        "name": "mixed",
        "objects": [
            { "type": "box", "size": [1, 1, 1], "position": [0, 0.5, 0], "mass": 0 },
            { "type": "box", "size": [0.5, 0.5, 0.5], "position": [0, 3, 0], "mass": 2.0 }
        ]
    })");

    SceneLoader loader;
    auto data = loader.Load(GetTestPath("mixed.json"));

    EXPECT_EQ(data.objects.size(), 2);
    EXPECT_TRUE(data.objects[0]->IsFixed());    // mass=0 => static
    EXPECT_FALSE(data.objects[1]->IsFixed());   // mass>0 => dynamic
}

TEST_F(SceneLoaderTest, SaveAndReload) {
    SceneData original;
    original.name = "saved_scene";
    original.gravity = {0, -4.0, 0};
    original.groundFriction = 0.8;
    original.robotSpawn = {1.0, 0.5, 2.0};

    SceneLoader loader;
    std::string savePath = GetTestPath("saved.json");
    loader.Save(savePath, original);

    auto loaded = loader.Load(savePath);
    EXPECT_EQ(loaded.name, "saved_scene");
    EXPECT_NEAR(loaded.gravity[1], -4.0, 1e-6);
    EXPECT_NEAR(loaded.groundFriction, 0.8, 1e-6);
    EXPECT_NEAR(loaded.robotSpawn[0], 1.0, 1e-6);
    EXPECT_NEAR(loaded.robotSpawn[2], 2.0, 1e-6);
}

TEST_F(SceneLoaderTest, CreateDefault) {
    auto data = SceneLoader::CreateDefault();
    EXPECT_EQ(data.name, "default");
    EXPECT_TRUE(data.hasGround);
    EXPECT_EQ(data.objects.size(), 0);
}
