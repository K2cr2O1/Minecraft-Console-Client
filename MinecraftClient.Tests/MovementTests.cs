using MinecraftClient.Mapping;
using Xunit;

namespace MinecraftClient.Tests
{
    public class MovementTests
    {
        private const int FeetY = 11;

        private static World BuildWorld()
        {
            World world = new();
            world[0, 0] = new ChunkColumn { FullyLoaded = true };
            return world;
        }

        // Palette112 maps old pre-1.13 block IDs to Materials; 1 is Stone.
        // Material.Stone cannot be cast directly: the enum is alphabetically ordered.
        private static Block Stone => new((short)1, 0);

        private static void FillGround(World world, int x1, int z1, int x2, int z2)
        {
            for (int x = x1; x <= x2; x++)
                for (int z = z1; z <= z2; z++)
                    world.SetBlock(new Location(x, FeetY - 1, z), Stone);
        }

        private static void FillWall(World world, int x1, int z1, int x2, int z2, int height = 2)
        {
            for (int x = x1; x <= x2; x++)
                for (int z = z1; z <= z2; z++)
                    for (int y = FeetY; y < FeetY + height; y++)
                        world.SetBlock(new Location(x, y, z), Stone);
        }

        private static Queue<Location> Waypoints(params (double X, double Z)[] points)
        {
            Queue<Location> queue = new();
            foreach ((double x, double z) in points)
                queue.Enqueue(new Location(x, FeetY, z));
            return queue;
        }

        [Fact]
        public void CanTravelStraight_OpenGround_ReturnsTrue()
        {
            World world = BuildWorld();
            FillGround(world, -2, -2, 5, 5);

            bool result = Movement.CanTravelStraight(
                world,
                new Location(0.5, FeetY, 0.5),
                new Location(3.5, FeetY, 3.5));

            Assert.True(result);
        }

        [Fact]
        public void CanTravelStraight_WallOnSegment_ReturnsFalse()
        {
            World world = BuildWorld();
            FillGround(world, -2, -2, 5, 5);
            // Wall on the straight line from (0.5, 0.5) to (1.5, 2.5)
            world.SetBlock(new Location(1, FeetY, 1), Stone);

            bool result = Movement.CanTravelStraight(
                world,
                new Location(0.5, FeetY, 0.5),
                new Location(1.5, FeetY, 2.5));

            Assert.False(result);
        }

        [Fact]
        public void SimplifyPath_StraightLine_CollapsesToEndpoints()
        {
            World world = BuildWorld();
            FillGround(world, -1, -1, 5, 5);
            Queue<Location> path = Waypoints((0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (3.5, 0.5));

            Queue<Location> simplified = Movement.SimplifyPath(world, path);

            Assert.Equal(2, simplified.Count);
            Assert.Equal(0.5, simplified.Peek().X, 3);
            Assert.Equal(3.5, simplified.Last().X, 3);
        }

        [Fact]
        public void SimplifyPath_OpenDiagonal_CollapsesToEndpoints()
        {
            World world = BuildWorld();
            FillGround(world, -1, -1, 5, 5);
            Queue<Location> path = Waypoints((0.5, 0.5), (1.5, 1.5), (2.5, 2.5), (3.5, 3.5));

            Queue<Location> simplified = Movement.SimplifyPath(world, path);

            Assert.Equal(2, simplified.Count);
            Assert.Equal(0.5, simplified.Peek().Z, 3);
            Assert.Equal(3.5, simplified.Last().Z, 3);
        }

        [Fact]
        public void SimplifyPath_LCorner_KeepsCornerWaypoint()
        {
            World world = BuildWorld();
            // L-shaped corridor: cells (0,3), (1,3), (1,4), (1,5)
            FillGround(world, 0, 3, 1, 5);
            // Outside walls: block the diagonal shortcut through (0,4)
            FillWall(world, 0, 4, 0, 4);
            FillWall(world, -1, 3, -1, 5);
            FillWall(world, 0, 2, 2, 2);
            FillWall(world, 2, 3, 2, 5);

            Queue<Location> path = Waypoints((0.5, 3.5), (1.5, 3.5), (1.5, 4.5), (1.5, 5.5));
            Queue<Location> simplified = Movement.SimplifyPath(world, path);

            // The turn waypoint (1.5, 3.5) must be preserved because the diagonal
            // shortcut through (0,4) is blocked; the mid-corridor waypoint (1.5, 4.5)
            // is redundant and must be collapsed.
            Assert.Equal(3, simplified.Count);
            Assert.Contains(simplified, waypoint => waypoint.X == 1.5 && waypoint.Z == 3.5);
            Assert.DoesNotContain(simplified, waypoint => waypoint.X == 1.5 && waypoint.Z == 4.5);
        }

        [Fact]
        public void SimplifyPath_NarrowCorridor_DoesNotCutThroughWall()
        {
            World world = BuildWorld();
            // 1-wide corridor along z=3, then a 90-degree turn at x=2
            FillGround(world, 0, 3, 2, 3);
            FillGround(world, 2, 3, 2, 5);
            FillWall(world, 0, 2, 2, 2);
            FillWall(world, 0, 4, 1, 4);
            FillWall(world, 2, 6, 2, 6);
            FillWall(world, 3, 3, 3, 5);

            Queue<Location> path = Waypoints((0.5, 3.5), (1.5, 3.5), (2.5, 3.5), (2.5, 4.5), (2.5, 5.5));
            Queue<Location> simplified = Movement.SimplifyPath(world, path);

            Assert.Contains(simplified, waypoint => waypoint.X == 2.5 && waypoint.Z == 3.5);
        }

        [Fact]
        public void CalculatePath_StraightCorridor_ReturnsSmoothedPath()
        {
            World world = BuildWorld();
            // 1-wide corridor from x=1 to x=4 along z=3
            FillGround(world, 1, 3, 4, 3);
            FillWall(world, 1, 2, 4, 2);
            FillWall(world, 1, 4, 4, 4);
            world.SetBlock(new Location(0, FeetY - 1, 3), Stone);
            world.SetBlock(new Location(5, FeetY - 1, 3), Stone);

            Location start = new(1.5, FeetY, 3.5);
            Location goal = new(4.5, FeetY, 3.5);
            Queue<Location>? path = Movement.CalculatePath(
                world, start, goal, allowUnsafe: false, maxOffset: 0, minOffset: 0,
                TimeSpan.FromSeconds(5));

            Assert.NotNull(path);
            Assert.True(path!.Count <= 3, $"Expected smoothed path (<=3 waypoints), got {path.Count}");
        }
    }
}
