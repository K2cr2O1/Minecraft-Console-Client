using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace MinecraftClient.Mapping
{
    /// <summary>
    /// Allows moving through a Minecraft world
    /// </summary>
    public static class Movement
    {
        /// <summary>
        /// Number of nodes expanded by the most recent CalculatePath call (diagnostics).
        /// </summary>
        public static long LastExpandedNodes;

        /// <summary>
        /// Integer scale for pathfinding costs and heuristics (1 block = 1000).
        /// </summary>
        private const int MoveCostScale = 1000;
        /// <summary>
        /// Max fall distance allowed in risk mode (-f): landing platforms up to
        /// this many blocks below are acceptable, provably bottomless cells are not.
        /// </summary>
        private const int UnsafeFallDepth = 5;

        /* ========= PATHFINDING METHODS ========= */

        /// <summary>
        /// Handle movements due to gravity
        /// </summary>
        /// <param name="world">World the player is currently located in</param>
        /// <param name="location">Location the player is currently at</param>
        /// <param name="motionY">Current vertical motion speed</param>
        /// <returns>Updated location after applying gravity</returns>
        public static Location HandleGravity(World world, Location location, ref double motionY)
        {
            if (Settings.InternalConfig.GravityEnabled)
            {
                Location onFoots = new(location.X, Math.Floor(location.Y), location.Z);
                Location belowFoots = Move(location, Direction.Down);
                if (location.Y > Math.Truncate(location.Y) + 0.0001)
                {
                    belowFoots = location;
                    belowFoots.Y = Math.Truncate(location.Y);
                }

                if (!IsOnGround(world, location) && !IsSwimming(world, location))
                {
                    while (!IsOnGround(world, belowFoots) && belowFoots.Y >= 1 + World.GetDimension().minY)
                        belowFoots = Move(belowFoots, Direction.Down);
                    location = Move2Steps(location, belowFoots, ref motionY, true).Dequeue();
                }
                else if (!world.GetBlock(onFoots).Type.IsSolid())
                    location = Move2Steps(location, onFoots, ref motionY, true).Dequeue();
            }

            return location;
        }

        /// <summary>
        /// Return a list of possible moves for the player
        /// </summary>
        /// <param name="world">World the player is currently located in</param>
        /// <param name="originLocation">Location the player is currently at</param>
        /// <param name="allowUnsafe">Allow possible but unsafe locations</param>
        /// <returns>A list of new locations the player can move to</returns>
        public static IEnumerable<Location> GetAvailableMoves(World world, Location originLocation,
            bool allowUnsafe = false)
        {
            Location location = originLocation.ToCenter();
            List<Location> availableMoves = new();
            if (IsOnGround(world, location) || IsSwimming(world, location))
            {
                foreach (Direction dir in Enum.GetValues(typeof(Direction)))
                {
                    Location dest = Move(location, dir);
                    bool diagonalSafe = !IsDiagonal(dir)
                        || (IsSafe(world, Move(location, DiagonalCardinalA(dir)))
                            && IsSafe(world, Move(location, DiagonalCardinalB(dir))));
                    if (CanMove(world, location, dir) && (allowUnsafe || (IsSafe(world, dest) && diagonalSafe)))
                        availableMoves.Add(dest);
                }

                // Step-up: jump onto a 1-high solid block in an adjacent cell.
                // Without this diagonal-up transition the pathfinder can only plan
                // same-column jumps into mid-air, which the bot can never land on,
                // so it can never climb stairs or escape pits.
                foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
                {
                    Location stepUp = Move(Move(location, dir), Direction.Up);
                    if (CanStepUp(world, location, dir) && (allowUnsafe || IsSafe(world, stepUp)))
                        availableMoves.Add(stepUp);
                }
            }
            else
            {
                foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
                    if (CanMove(world, location, dir) && IsOnGround(world, Move(location, dir)) &&
                        (allowUnsafe || IsSafe(world, Move(location, dir))))
                        availableMoves.Add(Move(location, dir));
                availableMoves.Add(Move(location, Direction.Down));
            }

            return availableMoves;
        }

        private static bool IsDiagonal(Direction direction)
        {
            return direction == Direction.NorthEast || direction == Direction.SouthEast
                || direction == Direction.SouthWest || direction == Direction.NorthWest;
        }

        private static Direction DiagonalCardinalA(Direction direction)
        {
            return direction switch
            {
                Direction.NorthEast => Direction.North,
                Direction.SouthEast => Direction.South,
                Direction.SouthWest => Direction.South,
                Direction.NorthWest => Direction.North,
                _ => throw new ArgumentException("Not a diagonal direction", nameof(direction))
            };
        }

        private static Direction DiagonalCardinalB(Direction direction)
        {
            return direction switch
            {
                Direction.NorthEast => Direction.East,
                Direction.SouthEast => Direction.East,
                Direction.SouthWest => Direction.West,
                Direction.NorthWest => Direction.West,
                _ => throw new ArgumentException("Not a diagonal direction", nameof(direction))
            };
        }

        /// <summary>
        /// Decompose a single move from a block to another into several steps
        /// </summary>
        /// <remarks>
        /// Allows moving by little steps instead or directly moving between blocks,
        /// which would be rejected by anti-cheat plugins anyway.
        /// </remarks>
        /// <param name="start">Start location</param>
        /// <param name="goal">Destination location</param>
        /// <param name="motionY">Current vertical motion speed</param>
        /// <param name="falling">Specify if performing falling steps</param>
        /// <param name="stepsByBlock">Amount of steps by block</param>
        /// <returns>A list of locations corresponding to the requested steps</returns>
        public static Queue<Location> Move2Steps(Location start, Location goal, ref double motionY,
            bool falling = false, int stepsByBlock = 8)
        {
            if (stepsByBlock <= 0)
                stepsByBlock = 1;

            if (falling)
            {
                //Use MC-Like falling algorithm
                double y = start.Y;
                Queue<Location> fallSteps = new();
                fallSteps.Enqueue(start);
                motionY -= 0.08D;
                motionY *= 0.9800000190734863D;
                y += motionY;

                if (y < goal.Y)
                    return new Queue<Location>(new[] { goal });

                return new Queue<Location>(new[] { new Location(start.X, y, start.Z) });
            }
            else
            {
                //Regular MCC moving algorithm
                motionY = 0; //Reset motion speed
                double totalStepsDouble = start.Distance(goal) * stepsByBlock;
                int totalSteps = (int)Math.Ceiling(totalStepsDouble);
                Location step = (goal - start) / totalSteps;

                if (totalStepsDouble >= 1)
                {
                    Queue<Location> movementSteps = new();
                    for (int i = 1; i <= totalSteps; i++)
                        movementSteps.Enqueue(start + step * i);
                    return movementSteps;
                }
                else
                    return new Queue<Location>(new[] { goal });
            }
        }

        /// <summary>
        /// Calculate a path from the start location to the destination location
        /// </summary>
        /// <remarks>
        /// Based on the A* pathfinding algorithm described on Wikipedia
        /// </remarks>
        /// <see href="https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode"/>
        /// <param name="world">World</param>
        /// <param name="start">Start location</param>
        /// <param name="goal">Destination location</param>
        /// <param name="allowUnsafe">Allow possible but unsafe locations</param>
        /// <param name="maxOffset">If no valid path can be found, also allow locations within specified distance of destination</param>
        /// <param name="minOffset">Do not get closer of destination than specified distance</param>
        /// <param name="timeout">How long to wait before stopping computation</param>
        /// <remarks>When location is unreachable, computation will reach timeout, then optionally fallback to a close location within maxOffset</remarks>
        /// <returns>A list of locations, or null if calculation failed</returns>
        public static Queue<Location>? CalculatePath(World world, Location start, Location goal, bool allowUnsafe,
            int maxOffset, int minOffset, TimeSpan timeout)
        {
            CancellationTokenSource cts = new();
            Task<Queue<Location>?> pathfindingTask = Task.Factory.StartNew(() =>
                CalculatePath(world, start, goal, allowUnsafe, maxOffset, minOffset, cts.Token));
            pathfindingTask.Wait(timeout);
            if (!pathfindingTask.IsCompleted)
            {
                cts.Cancel();
                pathfindingTask.Wait();
            }

            return pathfindingTask.Result;
        }

        /// <summary>
        /// Calculate a path from the start location to the destination location
        /// </summary>
        /// <remarks>
        /// Based on the A* pathfinding algorithm described on Wikipedia
        /// </remarks>
        /// <see href="https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode"/>
        /// <param name="world">World</param>
        /// <param name="start">Start location</param>
        /// <param name="goal">Destination location</param>
        /// <param name="allowUnsafe">Allow possible but unsafe locations</param>
        /// <param name="maxOffset">If no valid path can be found, also allow locations within specified distance of destination</param>
        /// <param name="minOffset">Do not get closer of destination than specified distance</param>
        /// <param name="ct">Token for stopping computation after a certain time</param>
        /// <returns>A list of locations, or null if calculation failed</returns>
        public static Queue<Location>? CalculatePath(World world, Location start, Location goal, bool allowUnsafe,
            int maxOffset, int minOffset, CancellationToken ct, int maxNodes = 0)
        {
            // This is a bad configuration
            if (minOffset > maxOffset)
                throw new ArgumentException("minOffset must be lower or equal to maxOffset", nameof(minOffset));

            // Bidirectional A*: forward JPS from the start, backward search from the
            // goal. Used only for unsafe (exploration) searches without offsets,
            // where the reverse generator exactly mirrors the forward edges.
            if (allowUnsafe && maxOffset <= 0 && minOffset <= 0)
                return CalculatePathBidirectional(world, start, goal, ct, maxNodes);

            // Round start coordinates for easier calculation
            Location startLower = start.ToFloor();
            Location goalLower = goal.ToFloor();

            // H/G scores are real distances scaled by MoveCostScale, so offsets
            // must use the same scale.
            minOffset *= MoveCostScale;
            maxOffset *= MoveCostScale;

            // Prepare variables and datastructures for A*

            // Dictionary that contains the relation between all coordinates and resolves the final path
            Dictionary<Location, Location> cameFrom = new();
            // Priority queue of (location, gScore) ordered by fScore; stale entries
            // are skipped via lazy deletion against gScoreDict.
            PriorityQueue<(Location Loc, int G), int> openSet = new();
            // Dictionary to keep track of the G-Score of every location
            Dictionary<Location, int> gScoreDict = new();
            Location? closestNode = null;
            int closestH = int.MaxValue;

            // Set start values for variables
            LastExpandedNodes = 0;
            openSet.Enqueue((startLower, 0), Heuristic(startLower, goalLower));
            gScoreDict[startLower] = 0;
            Location current = startLower;

            // Start of A*

            // Execute while we have nodes to process and we are not cancelled
            while (openSet.Count > 0 && !ct.IsCancellationRequested)
            {
                // Pop the lowest F-score entry; skip stale ones (lazy deletion)
                if (!openSet.TryDequeue(out var entry, out _))
                    break;
                if (!gScoreDict.TryGetValue(entry.Loc, out int entryG) || entryG != entry.G)
                    continue;
                current = entry.Loc;
                LastExpandedNodes++;

                // Node budget: stop expanding when the search grows too large.
                // The best-progress node seen so far becomes a partial path so the
                // bot can walk to the loaded terrain edge and re-plan there
                // (incremental / on-the-way planning for very distant goals).
                if (maxNodes > 0 && LastExpandedNodes >= maxNodes)
                    break;

                int currentH = Heuristic(current, goalLower);
                if (currentH < closestH)
                {
                    closestH = currentH;
                    closestNode = current;
                }

                // Return if goal found and no maxOffset was given OR current node is between minOffset and maxOffset
                if ((current == goalLower && maxOffset <= 0) ||
                    (maxOffset > 0 && currentH >= minOffset && currentH <= maxOffset))
                    return SimplifyPath(world, ReconstructPath(cameFrom, current, start, goal));

                // Discover neighbored blocks
                foreach (Location neighbor in GetJumpPointNeighbors(world, current, goalLower, allowUnsafe))
                {
                    // If we are cancelled: break
                    if (ct.IsCancellationRequested)
                        break;

                    // tentative_GScore is the distance from start to the neighbor through current
                    int tentativeGScore = entryG + MoveCost(current, neighbor);

                    // If the neighbor is not in the GScoreDict OR its current tentativeGScore is lower than the previously saved one: 
                    if (!gScoreDict.TryGetValue(neighbor, out int existingGScore) ||
                        tentativeGScore < existingGScore)
                    {
                        // Save the new relation between the neighbored block and the current one
                        cameFrom[neighbor] = current;
                        gScoreDict[neighbor] = tentativeGScore;

                        openSet.Enqueue((neighbor, tentativeGScore), tentativeGScore + Heuristic(neighbor, goalLower));
                    }
                }
            }

            // Goal could not be reached. Set the path to the closest location if close enough
            if (closestNode is not null &&
                (maxOffset == int.MaxValue || closestH <= maxOffset))
                return SimplifyPath(world, ReconstructPath(cameFrom, closestNode.Value, start, goal));

            return null;
        }

        /// <summary>
        /// Bidirectional A*: expands from the start (JPS) and from the goal
        /// (reverse-neighbor expansion) until the frontiers meet. For long
        /// winding mazes this explores dramatically fewer nodes than a single
        /// forward search.
        /// </summary>
        private static Queue<Location>? CalculatePathBidirectional(
            World world, Location start, Location goal, CancellationToken ct, int maxNodes)
        {
            Location startLower = start.ToFloor();
            Location goalLower = goal.ToFloor();

            PriorityQueue<(Location Loc, int G), int> openForward = new();
            Dictionary<Location, Location> cameFromForward = new();
            Dictionary<Location, int> gForward = new();
            openForward.Enqueue((startLower, 0), Heuristic(startLower, goalLower));
            gForward[startLower] = 0;

            PriorityQueue<(Location Loc, int G), int> openBackward = new();
            Dictionary<Location, Location> cameFromBackward = new();
            Dictionary<Location, int> gBackward = new();
            openBackward.Enqueue((goalLower, 0), Heuristic(goalLower, startLower));
            gBackward[goalLower] = 0;

            HashSet<Location> closedForward = new();
            HashSet<Location> closedBackward = new();
            Location? meeting = null;
            int meetingCost = int.MaxValue;
            // Best forward-progress node (start side only, so it is always
            // reachable from the start): used to emit a partial path when the node
            // budget runs out before the frontiers meet.
            Location bestProgress = startLower;
            int bestProgressH = Heuristic(startLower, goalLower);

            while (openForward.Count > 0 && openBackward.Count > 0 && !ct.IsCancellationRequested)
            {
                if (maxNodes > 0 && LastExpandedNodes >= maxNodes)
                    break;

                if (!openForward.TryPeek(out _, out int fForward)
                    || !openBackward.TryPeek(out _, out int fBackward))
                    break;

                if (meeting is not null)
                {
                    int minF = Math.Min(fForward, fBackward);
                    if (minF >= meetingCost)
                        break;
                }

                if (fForward <= fBackward)
                {
                    if (!openForward.TryDequeue(out var entry, out _))
                        break;
                    if (!gForward.TryGetValue(entry.Loc, out int entryG) || entryG != entry.G)
                        continue;
                    if (!closedForward.Add(entry.Loc))
                        continue;
                    LastExpandedNodes++;
                    int h = Heuristic(entry.Loc, goalLower);
                    if (h < bestProgressH)
                    {
                        bestProgressH = h;
                        bestProgress = entry.Loc;
                    }

                    if (closedBackward.Contains(entry.Loc))
                    {
                        int cost = entryG + gBackward[entry.Loc];
                        if (cost < meetingCost) { meetingCost = cost; meeting = entry.Loc; }
                    }

                    foreach (Location neighbor in GetJumpPointNeighbors(world, entry.Loc, goalLower, allowUnsafe: true))
                    {
                        if (ct.IsCancellationRequested)
                            break;
                        int tentativeG = entryG + MoveCost(entry.Loc, neighbor);
                        if (!gForward.TryGetValue(neighbor, out int existing) || tentativeG < existing)
                        {
                            cameFromForward[neighbor] = entry.Loc;
                            gForward[neighbor] = tentativeG;
                            openForward.Enqueue((neighbor, tentativeG), tentativeG + Heuristic(neighbor, goalLower));
                        }
                    }
                }
                else
                {
                    if (!openBackward.TryDequeue(out var entry, out _))
                        break;
                    if (!gBackward.TryGetValue(entry.Loc, out int entryG) || entryG != entry.G)
                        continue;
                    if (!closedBackward.Add(entry.Loc))
                        continue;
                    LastExpandedNodes++;

                    if (closedForward.Contains(entry.Loc))
                    {
                        int cost = entryG + gForward[entry.Loc];
                        if (cost < meetingCost) { meetingCost = cost; meeting = entry.Loc; }
                    }

                    foreach (Location neighbor in GetReverseNeighbors(world, entry.Loc))
                    {
                        if (ct.IsCancellationRequested)
                            break;
                        int tentativeG = entryG + MoveCost(entry.Loc, neighbor);
                        if (!gBackward.TryGetValue(neighbor, out int existing) || tentativeG < existing)
                        {
                            cameFromBackward[neighbor] = entry.Loc;
                            gBackward[neighbor] = tentativeG;
                            openBackward.Enqueue((neighbor, tentativeG), tentativeG + Heuristic(neighbor, startLower));
                        }
                    }
                }
            }

            if (meeting is null)
            {
                // Budget exhausted before the frontiers met: return the best
                // reachable prefix so the bot makes progress toward the goal and
                // re-plans from there with fresher chunk data.
                if (maxNodes > 0 && bestProgress != startLower)
                    return SimplifyPath(world, ReconstructPath(cameFromForward, bestProgress, start, goal));
                return null;
            }

            // Reconstruct: start -> meeting (forward), then meeting -> goal (backward)
            List<Location> path = new();
            Location cur = meeting.Value;
            path.Add(cur);
            while (cameFromForward.TryGetValue(cur, out Location prev))
            {
                path.Add(prev);
                cur = prev;
            }
            path.Reverse();
            cur = meeting.Value;
            while (cameFromBackward.TryGetValue(cur, out Location next))
            {
                path.Add(next);
                cur = next;
            }

            // End at the exact requested position instead of the cell floor
            if (path[^1] != goal && goal.DistanceSquared(path[^1]) <= 2.0)
                path[^1] = goal;

            return SimplifyPath(world, new Queue<Location>(path));
        }

        /// <summary>
        /// Reverse-neighbor generation for the backward search: all cells that have
        /// a forward edge into `node` (horizontal moves, step-up sources, and cells
        /// that can fall or climb down onto it).
        /// </summary>
        private static IEnumerable<Location> GetReverseNeighbors(World world, Location node)
        {
            // 1. Horizontal reverse: M -> node where M = node - dir
            foreach (Direction dir in new[]
            {
                Direction.East, Direction.West, Direction.North, Direction.South,
                Direction.NorthEast, Direction.SouthEast, Direction.SouthWest, Direction.NorthWest
            })
            {
                Location source = Move(node, Opposite(dir));
                // Mirror the forward-edge rule used in risk mode: the destination
                // (node) must have a landing within UnsafeFallDepth or unknown
                // terrain; provably bottomless cells have no reverse edge either.
                if (CanWalkHorizontally(world, source, dir)
                    && HasSupportWithin(world, node, UnsafeFallDepth))
                    yield return source;
            }

            // 2. Step-up reverse: source = node - dir - Up where the forward
            //    step-up edge source -> node exists.
            foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
            {
                Location source = Move(Move(node, Opposite(dir)), Direction.Down);
                if (CanStepUp(world, source, dir)
                    && Move(Move(source, dir), Direction.Up) == node)
                    yield return source;
            }

            // 3. Fall/climb reverse: the cell directly above can drop onto node.
            Location above = Move(node, Direction.Up);
            if (!IsOnGround(world, above) && !IsSwimming(world, above)
                && HasSupportWithin(world, node, UnsafeFallDepth))
                yield return above;
        }

        /// <summary>
        /// Real movement cost between two adjacent locations, scaled to integers.
        /// Cardinal/vertical moves cost 1000, diagonals cost ~1414.
        /// </summary>
        private static int MoveCost(Location from, Location to)
        {
            double dx = to.X - from.X;
            double dy = to.Y - from.Y;
            double dz = to.Z - from.Z;
            return (int)Math.Round(MoveCostScale * Math.Sqrt(dx * dx + dy * dy + dz * dz));
        }

        /// <summary>
        /// Admissible and consistent heuristic: scaled Euclidean distance.
        /// The previous squared-distance heuristic massively overestimated H
        /// (e.g. 26 blocks away scored 712 instead of ~26), turning A* into a
        /// greedy search that timed out on large mazes.
        /// </summary>
        private static int Heuristic(Location from, Location to)
        {
            double dx = Math.Abs(to.X - from.X);
            double dy = Math.Abs(to.Y - from.Y);
            double dz = Math.Abs(to.Z - from.Z);
            return (int)Math.Round(MoveCostScale * Math.Sqrt(dx * dx + dy * dy + dz * dz));
        }

        /// <summary>
        /// JPS (Jump Point Search) neighbor generation: horizontal 8-direction jump
        /// points plus per-node vertical transitions (step-up, fall, climb).
        /// On open grid layers this prunes almost all intermediate cells, which is
        /// what makes large multi-floor mazes searchable within a few seconds.
        /// </summary>
        private static IEnumerable<Location> GetJumpPointNeighbors(
            World world, Location node, Location goal, bool allowUnsafe)
        {
            // Horizontal JPS in all 8 directions
            foreach (Direction dir in new[]
            {
                Direction.East, Direction.West, Direction.North, Direction.South,
                Direction.NorthEast, Direction.SouthEast, Direction.SouthWest, Direction.NorthWest
            })
            {
                Location? jumpPoint = Jump(world, node, dir, goal);
                if (jumpPoint is not null)
                    yield return jumpPoint.Value;
            }

            // Walking off an edge into an unsupported air cell (starts a fall).
            // Safe mode requires a landing within IsSafe's 3-block rule. Risk mode
            // (-f) relaxes that to UnsafeFallDepth, but still refuses cells that are
            // provably bottomless in loaded terrain: those are voids, not drops.
            // Not-yet-loaded cells are treated as unknown and allowed, matching the
            // exploration semantics of -f (terrain streams in as the bot walks).
            foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
            {
                Location side = Move(node, dir);
                if (CanWalkHorizontally(world, node, dir) && !IsWalkableColumn(world, side))
                {
                    if (allowUnsafe ? HasSupportWithin(world, side, UnsafeFallDepth) : IsSafe(world, side))
                        yield return side;
                }
            }

            // Step-up transitions (1-high solid blocks)
            foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
            {
                Location stepUp = Move(Move(node, dir), Direction.Up);
                if (CanStepUp(world, node, dir) && (allowUnsafe || IsSafe(world, stepUp)))
                    yield return stepUp;
            }

            // Same-column climbing / swimming up
            if ((IsClimbing(world, node) || IsSwimming(world, node))
                && CanMove(world, node, Direction.Up)
                && (allowUnsafe || IsSafe(world, Move(node, Direction.Up))))
            {
                yield return Move(node, Direction.Up);
            }

            // Falling: continue downward while airborne (matches original behavior,
            // which let falls proceed and only filtered destinations by safety)
            if (!IsOnGround(world, node) && !IsSwimming(world, node))
                yield return Move(node, Direction.Down);
        }

        /// <summary>
        /// Jump in one direction to the next jump point: the goal, a forced
        /// neighbor, a vertical transition, or the edge of the loaded map.
        /// </summary>
        private static Location? Jump(World world, Location from, Direction dir, Location goal)
        {
            Location next = Move(from, dir);
            if (!PlayerFitsHere(world, next))
                return null;
            if (IsDiagonal(dir)
                && (!PlayerFitsHere(world, Move(from, DiagonalCardinalA(dir)))
                    || !PlayerFitsHere(world, Move(from, DiagonalCardinalB(dir)))))
                return null;
            return JumpRecursive(world, next, dir, goal);
        }

        private static Location? JumpRecursive(World world, Location next, Direction dir, Location goal)
        {
            // Solid or otherwise unenterable cell: the jump stops with no jump point
            if (!PlayerFitsHere(world, next))
                return null;

            if (next == goal)
                return next;

            // A cell the body fits in but that has no ground support within the safe
            // fall distance (or is not yet loaded) is an edge/void transition: stop
            // the jump there so the search can enter it and start falling.
            if (!IsWalkableColumn(world, next))
                return next;

            if (HasForcedNeighbor(world, next, dir))
                return next;

            if (IsDiagonal(dir))
            {
                // A diagonal jump must also stop where either cardinal component
                // would find a jump point.
                if (JumpRecursive(world, Move(next, DiagonalCardinalA(dir)), DiagonalCardinalA(dir), goal) != null
                    || JumpRecursive(world, Move(next, DiagonalCardinalB(dir)), DiagonalCardinalB(dir), goal) != null)
                    return next;
            }

            // Stop at cells where a vertical transition is possible: the bot must
            // evaluate step-ups/climbing there instead of jumping past them.
            if (HasVerticalTransition(world, next))
                return next;

            if (!CanWalkHorizontally(world, next, dir))
                return null;
            return JumpRecursive(world, Move(next, dir), dir, goal);
        }

        /// <summary>
        /// Whether the player can move horizontally from `from` into the cell in
        /// `dir`: body must fit (and for diagonals, both cardinal cells too) and
        /// the destination must be inside a fully loaded chunk.
        /// </summary>
        private static bool CanWalkHorizontally(World world, Location from, Direction dir)
        {
            Location dest = Move(from, dir);
            if (IsDiagonal(dir))
            {
                return PlayerFitsHere(world, dest)
                    && PlayerFitsHere(world, Move(from, DiagonalCardinalA(dir)))
                    && PlayerFitsHere(world, Move(from, DiagonalCardinalB(dir)));
            }
            return PlayerFitsHere(world, dest);
        }

        private static bool IsLoaded(World world, Location loc)
        {
            ChunkColumn? column = world.GetChunkColumn(loc);
            return column is not null && column.FullyLoaded;
        }

        /// <summary>
        /// Whether the block data for this cell actually exists in the cache.
        /// A column can be marked FullyLoaded while individual Y sections are
        /// still missing (GetBlock then reports Air for real terrain), so both
        /// must be checked before treating Air as proven void.
        /// </summary>
        private static bool HasBlockData(World world, Location loc)
        {
            ChunkColumn? column = world.GetChunkColumn(loc);
            return column is not null && column.GetChunk(loc) is not null;
        }

        private static bool IsOpenCell(World world, Location cell)
        {
            return IsLoaded(world, cell) && PlayerFitsHere(world, cell);
        }

        /// <summary>
        /// Classic JPS forced-neighbor test: a neighbor becomes newly reachable
        /// through `dir` because of an obstacle.
        /// </summary>
        private static bool HasForcedNeighbor(World world, Location node, Direction dir)
        {
            if (IsDiagonal(dir))
            {
                Location diagonal = Move(node, dir);
                return (!IsOpenCell(world, Move(node, DiagonalCardinalA(dir))) && IsOpenCell(world, diagonal))
                    || (!IsOpenCell(world, Move(node, DiagonalCardinalB(dir))) && IsOpenCell(world, diagonal));
            }
            else
            {
                Direction perp = Perpendicular(dir);
                Location diagonalA = Move(Move(node, dir), perp);
                Location diagonalB = Move(Move(node, dir), Opposite(perp));
                return (!IsOpenCell(world, Move(node, perp)) && IsOpenCell(world, diagonalA))
                    || (!IsOpenCell(world, Move(node, Opposite(perp))) && IsOpenCell(world, diagonalB));
            }
        }

        private static Direction Perpendicular(Direction direction)
        {
            return direction switch
            {
                Direction.East or Direction.West => Direction.North,
                Direction.North or Direction.South => Direction.East,
                _ => throw new ArgumentException("Not a cardinal direction", nameof(direction))
            };
        }

        private static Direction Opposite(Direction direction)
        {
            return direction switch
            {
                Direction.East => Direction.West,
                Direction.West => Direction.East,
                Direction.North => Direction.South,
                Direction.South => Direction.North,
                Direction.NorthEast => Direction.SouthWest,
                Direction.SouthWest => Direction.NorthEast,
                Direction.NorthWest => Direction.SouthEast,
                Direction.SouthEast => Direction.NorthWest,
                _ => throw new ArgumentException("Not a cardinal direction", nameof(direction))
            };
        }

        private static bool HasVerticalTransition(World world, Location node)
        {
            foreach (Direction dir in new[] { Direction.East, Direction.West, Direction.North, Direction.South })
            {
                if (CanStepUp(world, node, dir))
                    return true;
            }
            return IsClimbing(world, node) || IsSwimming(world, node);
        }

        /// <summary>
        /// Check whether the player can walk in a straight line between two locations
        /// on the same horizontal plane without colliding with solid blocks.
        /// Used for path smoothing and for skipping corner waypoints during execution.
        /// </summary>
        /// <param name="world">World</param>
        /// <param name="from">Start location (player feet position)</param>
        /// <param name="to">Destination location (player feet position)</param>
        /// <param name="playerRadius">Horizontal half-width of the player collision box</param>
        /// <returns>True when the straight segment is walkable</returns>
        public static bool CanTravelStraight(World world, Location from, Location to, double playerRadius = 0.3)
        {
            if (Math.Abs(from.Y - to.Y) > 0.01)
                return false; // Smoothing is limited to same-plane segments for now

            double dx = to.X - from.X;
            double dz = to.Z - from.Z;
            double distance = Math.Sqrt(dx * dx + dz * dz);
            if (distance < 1e-6)
                return true;

            // Sample the segment at least every quarter block
            int steps = Math.Max(1, (int)Math.Ceiling(distance / 0.25));
            for (int i = 0; i <= steps; i++)
            {
                double t = (double)i / steps;
                double px = from.X + dx * t;
                double pz = from.Z + dz * t;

                // Every block column touched by the player's collision box must be walkable
                int minX = (int)Math.Floor(px - playerRadius);
                int maxX = (int)Math.Floor(px + playerRadius);
                int minZ = (int)Math.Floor(pz - playerRadius);
                int maxZ = (int)Math.Floor(pz + playerRadius);
                for (int blockX = minX; blockX <= maxX; blockX++)
                {
                    for (int blockZ = minZ; blockZ <= maxZ; blockZ++)
                    {
                        if (!IsWalkableColumn(world, new Location(blockX, from.Y, blockZ)))
                            return false;
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// Remove unnecessary waypoints from a path by collapsing straight,
        /// collision-free segments. Keeps the path honest: corners are preserved
        /// whenever cutting them would clip a solid block.
        /// </summary>
        /// <param name="world">World</param>
        /// <param name="path">Path to simplify</param>
        /// <returns>Simplified path with the same start and end waypoints</returns>
        public static Queue<Location> SimplifyPath(World world, Queue<Location> path)
        {
            if (path.Count <= 2)
                return path;

            List<Location> waypoints = path.ToList();
            List<Location> simplified = new() { waypoints[0] };

            int anchorIndex = 0;
            for (int candidateIndex = 1; candidateIndex < waypoints.Count; candidateIndex++)
            {
                Location anchor = waypoints[anchorIndex];
                Location candidate = waypoints[candidateIndex];

                // Keep the previous waypoint when the direct segment is blocked
                if (!CanTravelStraight(world, anchor, candidate))
                {
                    int keptIndex = candidateIndex - 1;
                    if (keptIndex > anchorIndex)
                    {
                        simplified.Add(waypoints[keptIndex]);
                        anchorIndex = keptIndex;
                    }
                }
            }

            if (simplified.Last() != waypoints[^1])
                simplified.Add(waypoints[^1]);

            return new Queue<Location>(simplified);
        }

        /// <summary>
        /// Check whether the player's body (feet block and the block above) fits in a
        /// block column at the given feet position, and the column is fully loaded.
        /// </summary>
        private static bool IsWalkableColumn(World world, Location feet)
        {
            ChunkColumn? chunkColumn = world.GetChunkColumn(feet);
            if (chunkColumn is null || !chunkColumn.FullyLoaded)
                return false;

            Block feetBlock = world.GetBlock(feet);
            Block headBlock = world.GetBlock(Move(feet, Direction.Up));
            if (feetBlock.Type.IsSolid() || headBlock.Type.IsSolid())
                return false;

            // The column must have ground support within the safe fall distance
            // (same rule as IsSafe), otherwise smoothing would collapse a path
            // across a gap the player would fall through.
            return world.GetBlock(Move(feet, Direction.Down)).Type.IsSolid()
                || world.GetBlock(Move(feet, Direction.Down, 2)).Type.IsSolid()
                || world.GetBlock(Move(feet, Direction.Down, 3)).Type.IsSolid()
                || IsClimbing(world, Move(feet, Direction.Down))
                || IsClimbing(world, Move(feet, Direction.Down, 2))
                || IsClimbing(world, Move(feet, Direction.Down, 3));
        }

        /// <summary>
        /// Whether the column below `feet` has solid/climbable support within
        /// `depth` blocks. Unloaded terrain counts as unknown (supported), not as
        /// void, so -f exploration can walk into chunks that are still streaming in.
        /// </summary>
        private static bool HasSupportWithin(World world, Location feet, int depth)
        {
            if (!HasBlockData(world, feet))
                return true;
            for (int d = 1; d <= depth; d++)
            {
                Location below = Move(feet, Direction.Down, d);
                if (!HasBlockData(world, below))
                    return true;
                if (world.GetBlock(below).Type.IsSolid() || IsClimbing(world, below))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Helper function for CalculatePath(). Backtrack from goal to start to reconstruct a step-by-step path.
        /// </summary>
        /// <param name="cameFrom">The collection of Locations that leads back to the start</param>
        /// <param name="current">Endpoint of our later walk</param>
        /// <param name="start">Start location</param>
        /// <param name="end">End location</param>
        /// <returns>the path that leads to current from the start position</returns>
        private static Queue<Location> ReconstructPath(Dictionary<Location, Location> cameFrom, Location current,
            Location start, Location end)
        {
            int midPathCnt = 0;
            List<Location> totalPath = new();

            // Move from the center of the block to the final position
            if (current != end && current == end.ToFloor())
                totalPath.Add(end);

            // Generate intermediate paths
            totalPath.Add(current.ToCenter());
            while (cameFrom.ContainsKey(current))
            {
                ++midPathCnt;
                current = cameFrom[current];
                totalPath.Add(current.ToCenter());
            }

            if (midPathCnt <= 2 && start.DistanceSquared(end) < 2.0)
                return new Queue<Location>(new[] { end });
            else
            {
                // Move to the center of the block first
                if (current != start && current == start.ToFloor())
                    totalPath.Add(start.ToCenter());

                totalPath.Reverse();
                return new Queue<Location>(totalPath);
            }
        }

        /// <summary>
        /// A datastructure to store Locations as Nodes and provide them in sorted and queued order.
        /// !!!
        /// CAN BE REPLACED WITH PriorityQueue IN .NET-6
        /// https://docs.microsoft.com/en-us/dotnet/api/system.collections.generic.priorityqueue-2?view=net-6.0
        /// !!!
        /// </summary>
        public class BinaryHeap
        {
            /// <summary>
            /// Represents a location and its attributes
            /// </summary>
            public record Node(int GScore, int HScore, Location Location)
            {
                public int FScore => HScore + GScore;
            }

            // List which contains all nodes in form of a Binary Heap
            private readonly List<Node> heapList;

            // Hashset for quick checks of locations included in the heap
            private readonly HashSet<Location> locationList;
            public Node? MinHScoreNode;

            public BinaryHeap()
            {
                heapList = new();
                locationList = new();
                MinHScoreNode = null;
            }

            /// <summary>
            /// Insert a new location in the heap
            /// </summary>
            /// <param name="newGScore">G-Score of the location</param>
            /// <param name="newHScore">H-Score of the location</param>
            /// <param name="loc">The location</param>
            public void Insert(int newGScore, int newHScore, Location loc)
            {
                // Begin at the end of the list
                int i = heapList.Count;

                // Temporarily save the node created with the parameters to allow comparisons
                Node newNode = new(newGScore, newHScore, loc);

                // Add new note to the end of the list
                heapList.Add(newNode);
                locationList.Add(loc);

                // Save node with the smallest H-Score => Distance to goal
                if (MinHScoreNode is null || newNode.HScore < MinHScoreNode.HScore)
                    MinHScoreNode = newNode;

                if (i == 0)
                    return;

                // There is no need of sorting for one node.
                // Go up the heap from child to parent and move parent down...
                // while we are not looking at the root node AND the new node has better attributes than the parent node ((i - 1) / 2)
                while (i > 0 && FirstNodeBetter(newNode /* Current Child */,
                           heapList[(i - 1) / 2] /* Corresponding Parent */))
                {
                    // Move parent down and replace current child -> New free space is created
                    heapList[i] = heapList[(i - 1) / 2];
                    // Select the next parent to check
                    i = (i - 1) / 2;
                }

                // Nodes were moved down at position I there is now a free space at the correct position for our new node:
                // Insert new node in position
                heapList[i] = newNode;
            }

            /// <summary>
            /// Obtain the root which represents the node the the best attributes currently
            /// </summary>
            /// <returns>node with the best attributes currently</returns>
            /// <exception cref="InvalidOperationException"></exception>
            public Node GetRootLocation()
            {
                // The heap is empty. There is nothing to return.
                if (heapList.Count == 0)
                    throw new InvalidOperationException("The heap is empty.");

                // Save the root node
                var rootNode = heapList[0];
                locationList.Remove(rootNode.Location);

                // Temporarirly store the last item's value.
                var lastNode = heapList[^1];

                // Remove the last value.
                heapList.RemoveAt(heapList.Count - 1);

                if (heapList.Count > 0)
                {
                    // Start at the first index.
                    var currentParentPos = 0;

                    // Go through the heap from root to bottom...
                    // Continue until the halfway point of the heap.
                    while (currentParentPos < heapList.Count / 2)
                    {
                        // Select the left child of the current parent
                        var currentChildPos = (2 * currentParentPos) + 1;

                        // If the currently selected child is not the last entry of the list AND right child has better attributes
                        if ((currentChildPos < heapList.Count - 1) && FirstNodeBetter(heapList[currentChildPos + 1],
                                heapList[currentChildPos]))
                        {
                            // Select the right child
                            currentChildPos++;
                        }

                        // If the last item is smaller than both siblings at the
                        // current height, break.
                        if (FirstNodeBetter(lastNode, heapList[currentChildPos]))
                            break;

                        // Move the item at index j up one level.
                        heapList[currentParentPos] = heapList[currentChildPos];
                        // Move index i to the appropriate branch.
                        currentParentPos = currentChildPos;
                    }

                    // Insert the last node into the currently free position
                    heapList[currentParentPos] = lastNode;
                }

                return rootNode;
            }

            /// <summary>
            /// Compares two nodes and evaluates their position to the goal.
            /// </summary>
            /// <param name="firstNode">First node to compare</param>
            /// <param name="secondNode">Second node to compare</param>
            /// <returns>True if the first node has a more promising position to the goal than the second</returns>
            private static bool FirstNodeBetter(Node firstNode, Node secondNode)
            {
                // Is the FScore smaller?
                return (firstNode.FScore < secondNode.FScore) ||
                       // If FScore is equal, evaluate the h-score
                       (firstNode.FScore == secondNode.FScore && firstNode.HScore < secondNode.HScore);
            }

            /// <summary>
            /// Get the size of the heap
            /// </summary>
            /// <returns>size of the heap</returns>
            public int Count()
            {
                return heapList.Count;
            }

            /// <summary>
            /// Check if the heap contains a node with a certain location
            /// </summary>
            /// <param name="loc">Location to check</param>
            /// <returns>true if a node with the given location is in the heap</returns>
            public bool ContainsLocation(Location loc)
            {
                return locationList.Contains(loc);
            }
        }

        /* ========= LOCATION PROPERTIES ========= */

        // TODO: Find a way to remove this Hack for Vines here.

        /// <summary>
        /// Check if the specified location is on the ground
        /// </summary>
        /// <param name="world">World for performing check</param>
        /// <param name="location">Location to check</param>
        /// <returns>True if the specified location is on the ground</returns>
        public static bool IsOnGround(World world, Location location)
        {
            ChunkColumn? chunkColumn = world.GetChunkColumn(location);
            if (chunkColumn is null || chunkColumn.FullyLoaded == false)
                return true; // avoid moving downward in a not loaded chunk

            Location down = Move(location, Direction.Down);
            Material currentMaterial = world.GetBlock(down).Type;

            var result = currentMaterial.IsSolid()
                         || currentMaterial == Material.TwistingVines || currentMaterial == Material.TwistingVinesPlant
                         || currentMaterial == Material.WeepingVines || currentMaterial == Material.WeepingVinesPlant
                         || currentMaterial == Material.Vine;

            var northCheck = 1 + Math.Floor(down.Z) - down.Z > 0.7;
            var eastCheck = down.X - Math.Floor(down.X) > 0.7;
            var southCheck = down.Z - Math.Floor(down.Z) > 0.7;
            var westCheck = 1 + Math.Floor(down.X) - down.X > 0.7;

            if (!result && northCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.North));

            if (!result && northCheck && eastCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.NorthEast));

            if (!result && eastCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.East));

            if (!result && eastCheck && southCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.SouthEast));

            if (!result && southCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.South));

            if (!result && southCheck && westCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.SouthWest));

            if (!result && westCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.West));

            if (!result && westCheck && northCheck)
                result |= IsSolidOrVine(world, Move(down, Direction.NorthWest));

            return result && (location.Y <= Math.Truncate(location.Y) + 0.0001);
        }

        private static bool IsSolidOrVine(World world, Location location)
        {
            var block = world.GetBlock(location);
            return block.Type.IsSolid()
                   || block.Type == Material.TwistingVines
                   || block.Type == Material.TwistingVinesPlant
                   || block.Type == Material.WeepingVines
                   || block.Type == Material.WeepingVinesPlant
                   || block.Type == Material.Vine;
        }

        /// <summary>
        /// Check if the specified location implies swimming
        /// </summary>
        /// <param name="world">World for performing check</param>
        /// <param name="location">Location to check</param>
        /// <returns>True if the specified location implies swimming</returns>
        private static bool IsSwimming(World world, Location location)
        {
            return world.GetBlock(location).Type.IsLiquid();
        }

        /// <summary>
        /// Check if the specified location can be climbed on
        /// </summary>
        /// <param name="world">World for performing check</param>
        /// <param name="location">Location to check</param>
        /// <returns>True if the specified location can be climbed on</returns>
        private static bool IsClimbing(World world, Location location)
        {
            return world.GetBlock(location).Type.CanBeClimbedOn();
        }

        /// <summary>
        /// Check if the specified location is safe
        /// </summary>
        /// <param name="world">World for performing check</param>
        /// <param name="location">Location to check</param>
        /// <returns>True if the destination location won't directly harm the player</returns>
        private static bool IsSafe(World world, Location location)
        {
            return
                //The destination feet block itself must be passable
                !world.GetBlock(location).Type.IsSolid()

                //No block that can harm the player
                && !world.GetBlock(location).Type.CanHarmPlayers()
                && !world.GetBlock(Move(location, Direction.Up)).Type.CanHarmPlayers()
                && !world.GetBlock(Move(location, Direction.Down)).Type.CanHarmPlayers()

                //No fall from a too high place
                && (world.GetBlock(Move(location, Direction.Down)).Type.IsSolid() ||
                    IsClimbing(world, Move(location, Direction.Down))
                    || world.GetBlock(Move(location, Direction.Down, 2)).Type.IsSolid() ||
                    IsClimbing(world, Move(location, Direction.Down, 2))
                    || world.GetBlock(Move(location, Direction.Down, 3)).Type.IsSolid() ||
                    IsClimbing(world, Move(location, Direction.Down, 3)))

                //Not an underwater location
                && !(world.GetBlock(Move(location, Direction.Up)).Type.IsLiquid());
        }

        /* ========= SIMPLE MOVEMENTS ========= */

        /// <summary>
        /// Check if the player can move in the specified direction
        /// </summary>
        /// <param name="world">World the player is currently located in</param>
        /// <param name="location">Location the player is currently at</param>
        /// <param name="direction">Direction the player is moving to</param>
        /// <returns>True if the player can move in the specified direction</returns>
        public static bool CanMove(World world, Location location, Direction direction)
        {
            switch (direction)
            {
                // Move vertical
                case Direction.Down:
                    return IsClimbing(world, Move(location, Direction.Down)) || !IsOnGround(world, location);
                case Direction.Up:
                    // Same-column vertical moves are only valid when climbing or
                    // swimming. A plain jump straight up lands on nothing: the
                    // destination has no support block below it. Climbing stairs or
                    // jumping out of pits uses the diagonal step-up moves instead.
                    if (IsClimbing(world, location))
                        return IsClimbing(world, Move(location, Direction.Up))
                            || (!world.GetBlock(Move(location, Direction.Up)).Type.IsSolid()
                                && !world.GetBlock(Move(Move(location, Direction.Up), Direction.Up)).Type.IsSolid());

                    return IsSwimming(world, location)
                        && !world.GetBlock(Move(location, Direction.Up)).Type.IsSolid()
                        && !world.GetBlock(Move(Move(location, Direction.Up), Direction.Up)).Type.IsSolid();

                // Move horizontal
                case Direction.East:
                case Direction.West:
                case Direction.South:
                case Direction.North:
                    return PlayerFitsHere(world, Move(location, direction));

                // Move diagonal
                case Direction.NorthEast:
                    return PlayerFitsHere(world, Move(location, Direction.North)) &&
                           PlayerFitsHere(world, Move(location, Direction.East)) &&
                           PlayerFitsHere(world, Move(location, direction));
                case Direction.SouthEast:
                    return PlayerFitsHere(world, Move(location, Direction.South)) &&
                           PlayerFitsHere(world, Move(location, Direction.East)) &&
                           PlayerFitsHere(world, Move(location, direction));
                case Direction.SouthWest:
                    return PlayerFitsHere(world, Move(location, Direction.South)) &&
                           PlayerFitsHere(world, Move(location, Direction.West)) &&
                           PlayerFitsHere(world, Move(location, direction));
                case Direction.NorthWest:
                    return PlayerFitsHere(world, Move(location, Direction.North)) &&
                           PlayerFitsHere(world, Move(location, Direction.West)) &&
                           PlayerFitsHere(world, Move(location, direction));

                default:
                throw new ArgumentException("Unknown direction", nameof(direction));
            }
        }

        /// <summary>
        /// Check whether the player can step up onto a 1-high solid block in the
        /// adjacent cell in the given direction: the step block provides support
        /// at the destination feet level, and the player body fits at the top.
        /// </summary>
        private static bool CanStepUp(World world, Location location, Direction direction)
        {
            if (!IsOnGround(world, location) && !IsSwimming(world, location))
                return false;

            Location stepBlock = Move(location, direction);
            Location destination = Move(stepBlock, Direction.Up);

            // The step must be a full solid block whose top is exactly one level up
            if (!world.GetBlock(stepBlock).Type.IsSolid())
                return false;

            // The player body must fit at the destination (feet block and head block)
            if (world.GetBlock(destination).Type.IsSolid()
                || world.GetBlock(Move(destination, Direction.Up)).Type.IsSolid())
                return false;

            return true;
        }

        /// <summary>
        /// Evaluates if a player fits in this location
        /// </summary>
        /// <param name="world">Current world</param>
        /// <param name="location">Location to check</param>
        /// <returns>True if a player is able to stand in this location</returns>
        public static bool PlayerFitsHere(World world, Location location)
        {
            var canClimb = IsClimbing(world, location) && IsClimbing(world, Move(location, Direction.Up));
            var isNotSolid = !world.GetBlock(location).Type.IsSolid() &&
                             !world.GetBlock(Move(location, Direction.Up)).Type.IsSolid();

            // Handle slabs
            int? protocolVersion = McClient.Instance?.GetProtocolVersion();
            if (!isNotSolid && protocolVersion is not null &&
                world.GetBlock(Move(location, Direction.Up)).IsTopSlab(protocolVersion.Value))
            {
                isNotSolid = true;
            }

            return canClimb || isNotSolid;
        }

        /// <summary>
        /// Get an updated location for moving in the specified direction
        /// </summary>
        /// <param name="location">Current location</param>
        /// <param name="direction">Direction to move to</param>
        /// <param name="length">Distance, in blocks</param>
        /// <returns>Updated location</returns>
        public static Location Move(Location location, Direction direction, int length = 1)
        {
            return location + Move(direction) * length;
        }

        /// <summary>
        /// Get a location delta for moving in the specified direction
        /// </summary>
        /// <param name="direction">Direction to move to</param>
        /// <returns>A location delta for moving in that direction</returns>
        private static Location Move(Direction direction)
        {
            return direction switch
            {
                // Move vertical
                Direction.Down => new Location(0, -1, 0),
                Direction.Up => new Location(0, 1, 0),

                // Move horizontal straight
                Direction.East => new Location(1, 0, 0),
                Direction.West => new Location(-1, 0, 0),
                Direction.South => new Location(0, 0, 1),
                Direction.North => new Location(0, 0, -1),

                // Move horizontal diagonal
                Direction.NorthEast => Move(Direction.North) + Move(Direction.East),
                Direction.SouthEast => Move(Direction.South) + Move(Direction.East),
                Direction.SouthWest => Move(Direction.South) + Move(Direction.West),
                Direction.NorthWest => Move(Direction.North) + Move(Direction.West),

                _ => throw new ArgumentException("Unknown direction", nameof(direction))
            };
        }

        /// <summary>
        /// Check that the chunks at both the start and destination locations have been loaded
        /// </summary>
        /// <param name="world">Current world</param>
        /// <param name="start">Start location</param>
        /// <param name="dest">Destination location</param>
        /// <returns>Is loading complete</returns>
        public static bool CheckChunkLoading(World world, Location start, Location dest)
        {
            var chunkColumn = world.GetChunkColumn(dest);
            if (chunkColumn is null || chunkColumn.FullyLoaded == false)
                return false;

            chunkColumn = world.GetChunkColumn(start);
            if (chunkColumn is null || chunkColumn.FullyLoaded == false)
                return false;

            return true;
        }
    }
}
