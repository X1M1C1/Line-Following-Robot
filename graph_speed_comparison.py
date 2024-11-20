import timeit
from graph_path_handling import set_graph as old_set_graph, dijkstra as old_dijkstra
from improved_graph_path_handling import set_graph as new_set_graph, dijkstra as new_dijkstra

def compare_dijkstra(n, runs=10):
    """
    Compares the execution times of the old and new Dijkstra implementations.

    Parameters:
    - n (int): Grid size (n x n).
    - runs (int): Number of times to run each implementation for averaging.

    Returns:
    - None: Prints timing results.
    """
    print(f"Comparing Dijkstra's Algorithm on a {n}x{n} grid:")

    # Generate graphs for both implementations
    old_graph = old_set_graph(n)
    new_graph = new_set_graph(n)
    start, end = 0, n * n - 1

    # Timing old implementation
    old_time = timeit.timeit(
        lambda: old_dijkstra(old_graph, start, end),
        number=runs
    )

    # Timing new implementation
    new_time = timeit.timeit(
        lambda: new_dijkstra(new_graph, start, end),
        number=runs
    )

    print(f"Old Implementation: {old_time / runs:.6f} seconds (average over {runs} runs)")
    print(f"New Implementation: {new_time / runs:.6f} seconds (average over {runs} runs)")

if __name__ == "__main__":
    grid_size = 20  # Adjust the grid size as needed
    runs = 10  # Number of repetitions for averaging
    compare_dijkstra(grid_size, runs)
