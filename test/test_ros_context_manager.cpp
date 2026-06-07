// test_ros_context_manager.cpp — Focused gtest for RosContextManager
//
// Verifies reference-counted acquire/release semantics:
//   - First acquire calls rclcpp::init (context becomes ok)
//   - Subsequent acquires only increment the count (no double-init)
//   - Release decrements the count; shutdown only at last release
//   - No premature shutdown when multiple plugins hold leases
//   - Safe re-init after full release cycle
//   - Double-release is a no-op

#include "RosContextManager.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using MujocoRosUtils::RosContextManager;
using MujocoRosUtils::RosContextLease;

class RosContextManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // If a previous test left rclcpp initialised, shut it down so each test
    // starts from a clean slate.
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
    // Reset the singleton between tests by forcing the count to zero.
    // Since the singleton's state persists across tests (it's a static local),
    // we drain releases until the count is zero.
    auto & mgr = RosContextManager::instance();
    while (mgr.ref_count() > 0)
    {
      mgr.release();
    }
  }

  void TearDown() override
  {
    // Ensure rclcpp is shut down after each test.
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
    auto & mgr = RosContextManager::instance();
    while (mgr.ref_count() > 0)
    {
      mgr.release();
    }
  }
};

// ---- Basic acquire / release ----

TEST_F(RosContextManagerTest, FirstAcquireInitialisesRclcpp)
{
  auto & mgr = RosContextManager::instance();
  EXPECT_FALSE(rclcpp::ok());
  EXPECT_EQ(mgr.ref_count(), 0);

  int count = mgr.acquire();
  EXPECT_EQ(count, 1);
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(RosContextManagerTest, SecondAcquireOnlyIncrementsCount)
{
  auto & mgr = RosContextManager::instance();
  mgr.acquire();  // count = 1, rclcpp::init called
  EXPECT_TRUE(rclcpp::ok());

  int count = mgr.acquire();  // count = 2, no double-init
  EXPECT_EQ(count, 2);
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(RosContextManagerTest, ManagerOwnedContextSurvivesIntermediateAcquire)
{
  auto & mgr = RosContextManager::instance();
  mgr.acquire();
  mgr.acquire();
  mgr.release();
  EXPECT_TRUE(rclcpp::ok());
  mgr.release();
  EXPECT_FALSE(rclcpp::ok());
}

TEST_F(RosContextManagerTest, ExternallyOwnedContextIsNotShutdown)
{
  rclcpp::init(0, nullptr);
  auto & mgr = RosContextManager::instance();
  mgr.acquire();
  mgr.release();
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(RosContextManagerTest, LeaseReleasesAtScopeExit)
{
  auto & mgr = RosContextManager::instance();
  {
    RosContextLease lease;
    lease.acquire();
    EXPECT_EQ(mgr.ref_count(), 1U);
    EXPECT_TRUE(rclcpp::ok());
  }
  EXPECT_EQ(mgr.ref_count(), 0U);
  EXPECT_FALSE(rclcpp::ok());
}

TEST_F(RosContextManagerTest, LastReleaseShutsDownRclcpp)
{
  auto & mgr = RosContextManager::instance();
  mgr.acquire();       // count = 1
  EXPECT_TRUE(rclcpp::ok());

  int count = mgr.release();  // count = 0, shutdown
  EXPECT_EQ(count, 0);
  EXPECT_FALSE(rclcpp::ok());
}

TEST_F(RosContextManagerTest, IntermediateReleaseDoesNotShutdown)
{
  auto & mgr = RosContextManager::instance();
  mgr.acquire();       // count = 1
  mgr.acquire();       // count = 2
  mgr.acquire();       // count = 3

  int count = mgr.release();  // count = 2
  EXPECT_EQ(count, 2);
  EXPECT_TRUE(rclcpp::ok()) << "rclcpp was shut down prematurely while 2 leases remain";

  mgr.release();       // count = 1
  EXPECT_TRUE(rclcpp::ok()) << "rclcpp was shut down prematurely while 1 lease remains";

  mgr.release();       // count = 0, shutdown
  EXPECT_FALSE(rclcpp::ok());
}

// ---- Prevents one plugin from killing another ----

TEST_F(RosContextManagerTest, OnePluginCannotShutdownWhileAnotherAlive)
{
  auto & mgr = RosContextManager::instance();

  // Simulate two plugins acquiring
  mgr.acquire();  // "plugin A"
  mgr.acquire();  // "plugin B"
  EXPECT_TRUE(rclcpp::ok());

  // Plugin A is destroyed — should NOT shut down rclcpp
  mgr.release();  // "plugin A destroyed"
  EXPECT_TRUE(rclcpp::ok()) << "Plugin A's destruction shut down rclcpp while Plugin B is alive";

  // Plugin B is destroyed — should shut down
  mgr.release();
  EXPECT_FALSE(rclcpp::ok());
}

// ---- Re-init after full release cycle ----

TEST_F(RosContextManagerTest, ReinitAfterFullRelease)
{
  auto & mgr = RosContextManager::instance();

  // First cycle
  mgr.acquire();
  EXPECT_TRUE(rclcpp::ok());
  mgr.release();
  EXPECT_FALSE(rclcpp::ok());

  // Second cycle
  int count = mgr.acquire();
  EXPECT_EQ(count, 1);
  EXPECT_TRUE(rclcpp::ok());

  mgr.release();
  EXPECT_FALSE(rclcpp::ok());
}

// ---- Double release is a no-op ----

TEST_F(RosContextManagerTest, DoubleReleaseIsNoOp)
{
  auto & mgr = RosContextManager::instance();
  mgr.acquire();
  mgr.release();

  EXPECT_EQ(mgr.ref_count(), 0);
  // Second release should not crash or go negative
  int count = mgr.release();
  EXPECT_EQ(count, 0);
}

// ---- Ref count is accurate ----

TEST_F(RosContextManagerTest, RefCountTracksAcquiresAndReleases)
{
  auto & mgr = RosContextManager::instance();

  EXPECT_EQ(mgr.ref_count(), 0);

  mgr.acquire();
  EXPECT_EQ(mgr.ref_count(), 1);

  mgr.acquire();
  mgr.acquire();
  EXPECT_EQ(mgr.ref_count(), 3);

  mgr.release();
  EXPECT_EQ(mgr.ref_count(), 2);

  mgr.release();
  mgr.release();
  EXPECT_EQ(mgr.ref_count(), 0);
}

// ---- is_ok reflects rclcpp state ----

TEST_F(RosContextManagerTest, IsOkReflectsRclcppState)
{
  auto & mgr = RosContextManager::instance();

  EXPECT_FALSE(mgr.is_ok());
  mgr.acquire();
  EXPECT_TRUE(mgr.is_ok());
  mgr.release();
  EXPECT_FALSE(mgr.is_ok());
}
