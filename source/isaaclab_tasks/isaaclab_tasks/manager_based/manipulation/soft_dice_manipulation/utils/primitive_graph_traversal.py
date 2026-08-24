from dataclasses import dataclass
from collections import deque
import random

# ============================================================
# Dice definition
# ============================================================

# Your actual opposite-face pairs:
OPPOSITE = {
    1: 6,
    6: 1,
    3: 5,
    5: 3,
    4: 2,
    2: 4,
}


@dataclass(frozen=True)
class Orientation:
    """
    Face currently occupying each spatial direction.

    Example:
        top=1
        bottom=6
        front=3
        back=5
        left=4
        right=2
    """
    top: int
    bottom: int
    front: int
    back: int
    left: int
    right: int

    def __str__(self):
        return (
            f"top={self.top}, bottom={self.bottom}, "
            f"front={self.front}, back={self.back}, "
            f"left={self.left}, right={self.right}"
        )


# ============================================================
# Validation
# ============================================================

def validate_orientation(s: Orientation):
    faces = {
        s.top,
        s.bottom,
        s.front,
        s.back,
        s.left,
        s.right,
    }

    if faces != {1, 2, 3, 4, 5, 6}:
        raise ValueError(
            f"Orientation must contain faces 1-6 exactly once. Got: {faces}"
        )

    if OPPOSITE[s.top] != s.bottom:
        raise ValueError(
            f"Top {s.top} and bottom {s.bottom} are not opposite."
        )

    if OPPOSITE[s.front] != s.back:
        raise ValueError(
            f"Front {s.front} and back {s.back} are not opposite."
        )

    if OPPOSITE[s.left] != s.right:
        raise ValueError(
            f"Left {s.left} and right {s.right} are not opposite."
        )


# ============================================================
# Primitive transitions
# ============================================================

def rotate_right(s: Orientation) -> Orientation:
    """
    Definition used here:
    the RIGHT face becomes the new TOP face.

           old right -> new top
           old top   -> new left
           old left  -> new bottom
           old bottom-> new right

    Front/back remain unchanged.
    """
    return Orientation(
        top=s.right,
        bottom=s.left,
        front=s.front,
        back=s.back,
        left=s.top,
        right=s.bottom,
    )


def rotate_left(s: Orientation) -> Orientation:
    """
    Definition used here:
    the LEFT face becomes the new TOP face.
    """
    return Orientation(
        top=s.left,
        bottom=s.right,
        front=s.front,
        back=s.back,
        left=s.bottom,
        right=s.top,
    )


def yaw_right(s: Orientation) -> Orientation:
    """
    Clockwise rotation when viewed from above.

    Old front -> new right.
    Top/bottom remain unchanged.
    """
    return Orientation(
        top=s.top,
        bottom=s.bottom,
        front=s.left,
        back=s.right,
        left=s.back,
        right=s.front,
    )


def yaw_left(s: Orientation) -> Orientation:
    """
    Counter-clockwise rotation when viewed from above.

    Old front -> new left.
    """
    return Orientation(
        top=s.top,
        bottom=s.bottom,
        front=s.right,
        back=s.left,
        left=s.front,
        right=s.back,
    )


ACTIONS = {
    "rotate_right": rotate_right,
    "rotate_left": rotate_left,
    "yaw_right": yaw_right,
    "yaw_left": yaw_left,
}


# ============================================================
# Build complete orientation state space
# ============================================================

def build_orientation_graph(initial_orientation: Orientation):
    """
    Starting from one physically valid orientation, apply all four
    primitives until every reachable cube orientation is found.

    A cube should have exactly 24 rotational orientations.
    """

    validate_orientation(initial_orientation)

    graph = {}
    queue = deque([initial_orientation])
    visited = {initial_orientation}

    while queue:
        state = queue.popleft()
        graph[state] = {}

        for action_name, transition_fn in ACTIONS.items():
            next_state = transition_fn(state)

            validate_orientation(next_state)

            graph[state][action_name] = next_state

            if next_state not in visited:
                visited.add(next_state)
                queue.append(next_state)

    return graph


# ============================================================
# BFS: find shortest sequence to put a given face on top
# ============================================================

def bfs_to_top_face(graph, start: Orientation, target_face: int):
    """
    Find the shortest primitive sequence that results in
    target_face being on top.

    The final front/left/right orientation does not matter.
    """

    if target_face not in {1, 2, 3, 4, 5, 6}:
        raise ValueError("Target face must be between 1 and 6.")

    queue = deque([(start, [])])
    visited = {start}

    while queue:
        state, path = queue.popleft()

        if state.top == target_face:
            return path, state

        for action, next_state in graph[state].items():
            if next_state not in visited:
                visited.add(next_state)
                queue.append(
                    (next_state, path + [action])
                )

    return None, None


# ============================================================
# Pretty-print path
# ============================================================

def print_solution(graph, start, path):
    state = start

    print("\nSTART")
    print(state)

    for i, action in enumerate(path, start=1):
        state = graph[state][action]

        print(f"\n{i}. {action}")
        print(state)

    print("\nFINAL")
    print(state)


# ============================================================
# RANDOMIZED TEST
# ============================================================

if __name__ == "__main__":

    # --------------------------------------------------------
    # One known valid orientation of the physical dice.
    #
    # Opposite pairs:
    #     1 <-> 6
    #     3 <-> 5
    #     4 <-> 2
    #
    # This orientation is only used to generate the complete
    # 24-state orientation graph.
    # --------------------------------------------------------

    reference_orientation = Orientation(
        top=1,
        bottom=6,
        front=3,
        back=5,
        left=4,
        right=2,
    )

    # --------------------------------------------------------
    # Build complete orientation graph
    # --------------------------------------------------------

    graph = build_orientation_graph(reference_orientation)

    print(f"Number of states: {len(graph)}")

    assert len(graph) == 24, (
        f"Expected 24 cube orientations, got {len(graph)}"
    )

    # --------------------------------------------------------
    # Randomly choose an initial orientation
    # --------------------------------------------------------

    initial = random.choice(list(graph.keys()))

    # --------------------------------------------------------
    # Randomly choose a target top face.
    #
    # Exclude the current top face so the solution is never
    # trivially an empty path.
    # --------------------------------------------------------

    possible_targets = [
        face
        for face in range(1, 7)
        if face != initial.top
    ]

    target_face = random.choice(possible_targets)

    # --------------------------------------------------------
    # Print random test problem
    # --------------------------------------------------------

    print("\n" + "=" * 70)
    print("RANDOM TEST")
    print("=" * 70)

    print("\nInitial orientation:")
    print(initial)

    print(f"\nTarget top face: {target_face}")

    # --------------------------------------------------------
    # BFS
    # --------------------------------------------------------

    path, final_state = bfs_to_top_face(
        graph,
        initial,
        target_face,
    )

    if path is None:
        raise RuntimeError(
            f"No path found from {initial} "
            f"to top face {target_face}"
        )

    print(f"\nShortest path: {path}")
    print(f"Number of primitives: {len(path)}")

    # --------------------------------------------------------
    # Show every intermediate orientation
    # --------------------------------------------------------

    print_solution(
        graph,
        initial,
        path,
    )

    # --------------------------------------------------------
    # Final sanity check
    # --------------------------------------------------------

    assert final_state.top == target_face, (
        f"BFS failed: expected top face {target_face}, "
        f"but got {final_state.top}"
    )

    print("\nBFS validation PASSED.")