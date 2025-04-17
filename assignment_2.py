import heapq
import time

def greedy_best_first_search(grid, start, goal):
    """
    Implementasi algoritma Greedy Best-First Search untuk navigasi ambulan
    
    Args:
        grid: Grid yang merepresentasikan peta kota
        start: Tuple (x, y) posisi awal ambulan
        goal: Tuple (x, y) posisi rumah sakit
    
    Returns:
        path: Jalur yang ditemukan
        visited_count: Jumlah node yang dikunjungi
        time_taken: Waktu eksekusi dalam milidetik
    """
    # Mencatat waktu mulai
    start_time = time.time()
    
    # Ukuran grid
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    
    # Fungsi heuristik - menggunakan jarak Manhattan
    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return abs(x2 - x1) + abs(y2 - y1)
    
    # Gerakan yang mungkin: atas, kanan, bawah, kiri
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    
    # Menyiapkan struktur data
    open_set = []  # Priority queue
    closed_set = set()
    visited_count = 0
    
    # Simpan jalur dengan dictionary parent
    parent = {}
    
    # Format: (heuristic_score, position)
    heapq.heappush(open_set, (heuristic(start), start))
    
    while open_set:
        # Mengambil node dengan skor heuristik terkecil
        _, current = heapq.heappop(open_set)
        visited_count += 1
        
        # Jika tujuan sudah ditemukan, rekonstruksi jalur dan kembalikan hasilnya
        if current == goal:
            # Rekonstruksi jalur
            path = []
            while current != start:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            
            time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
            return path, visited_count, time_taken
        
        # Tandai node saat ini sebagai sudah dikunjungi
        if current in closed_set:
            continue
        closed_set.add(current)
        
        # Jelajahi semua tetangga yang mungkin
        for dx, dy in directions:
            x, y = current
            nx, ny = x + dx, y + dy
            
            # Periksa apakah posisi valid
            if 0 <= nx < rows and 0 <= ny < cols:
                neighbor = (nx, ny)
                
                # Jika ada kemacetan (T), lewati
                if grid[nx][ny] == 'T':
                    continue
                
                # Jika sudah dikunjungi, lewati
                if neighbor in closed_set:
                    continue
                
                # Simpan parent untuk rekontruksi jalur
                parent[neighbor] = current
                
                # Tambahkan ke open set
                heapq.heappush(open_set, (heuristic(neighbor), neighbor))
    
    # Jika tidak ada jalur yang ditemukan
    time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
    return None, visited_count, time_taken

def a_star_search(grid, start, goal):
    """
    Implementasi algoritma A* untuk navigasi ambulan
    
    Args:
        grid: Grid yang merepresentasikan peta kota
        start: Tuple (x, y) posisi awal ambulan
        goal: Tuple (x, y) posisi rumah sakit
    
    Returns:
        path: Jalur yang ditemukan
        visited_count: Jumlah node yang dikunjungi
        time_taken: Waktu eksekusi dalam milidetik
    """
    # Mencatat waktu mulai
    start_time = time.time()
    
    # Ukuran grid
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    
    # Fungsi heuristik - menggunakan jarak Manhattan
    def heuristic(pos):
        x1, y1 = pos
        x2, y2 = goal
        return abs(x2 - x1) + abs(y2 - y1)
    
    # Gerakan yang mungkin: atas, kanan, bawah, kiri
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    
    # Menyiapkan struktur data
    open_set = []  # Priority queue
    closed_set = set()
    g_score = {start: 0}  # Biaya sebenarnya dari start ke sebuah node
    visited_count = 0
    
    # Simpan jalur dengan dictionary parent
    parent = {}
    
    # Format: (f_score, position)
    # f_score = g_score + heuristic
    heapq.heappush(open_set, (heuristic(start), start))
    
    while open_set:
        # Mengambil node dengan f_score terkecil
        _, current = heapq.heappop(open_set)
        visited_count += 1
        
        # Jika tujuan sudah ditemukan, rekonstruksi jalur dan kembalikan hasilnya
        if current == goal:
            # Rekonstruksi jalur
            path = []
            while current != start:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            
            time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
            return path, visited_count, time_taken
        
        # Tandai node saat ini sebagai sudah dikunjungi
        if current in closed_set:
            continue
        closed_set.add(current)
        
        # Jelajahi semua tetangga yang mungkin
        for dx, dy in directions:
            x, y = current
            nx, ny = x + dx, y + dy
            
            # Periksa apakah posisi valid
            if 0 <= nx < rows and 0 <= ny < cols:
                neighbor = (nx, ny)
                
                # Jika ada kemacetan (T), lewati
                if grid[nx][ny] == 'T':
                    continue
                
                # Jika sudah dikunjungi, lewati
                if neighbor in closed_set:
                    continue
                
                # Biaya gerakan
                cost = 1
                if grid[nx][ny] == '.':  # Jalan terbuka
                    cost = 1
                
                # Hitung g_score baru
                tentative_g_score = g_score.get(current, float('inf')) + cost
                
                # Jika kita menemukan jalur yang lebih baik ke neighbor
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # Simpan jalur yang lebih baik
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor)
                    
                    # Tambahkan ke open set
                    heapq.heappush(open_set, (f_score, neighbor))
    
    # Jika tidak ada jalur yang ditemukan
    time_taken = (time.time() - start_time) * 1000  # Konversi ke milidetik
    return None, visited_count, time_taken

# Contoh penggunaan
if __name__ == "__main__":
    # Contoh peta kota
    # S: Ambulan, H: Rumah sakit, T: Kemacetan, .: Jalan terbuka
    city_map = [
        [".", ".", ".", ".", ".", ".", ".", ".", "."],
        [".", "S", ".", ".", "T", "T", ".", ".", "."],
        [".", ".", ".", ".", "T", ".", ".", ".", "."],
        [".", "T", "T", "T", "T", ".", "T", "T", "."],
        [".", ".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", "T", "T", ".", "T", "T", ".", "."],
        [".", ".", ".", ".", ".", ".", "T", ".", "."],
        [".", "T", ".", "T", "T", ".", ".", "H", "."],
        [".", ".", ".", ".", ".", ".", ".", ".", "."]
    ]
    
    # Temukan posisi ambulan dan rumah sakit
    start_pos = None
    goal_pos = None
    
    for i in range(len(city_map)):
        for j in range(len(city_map[0])):
            if city_map[i][j] == "S":
                start_pos = (i, j)
            elif city_map[i][j] == "H":
                goal_pos = (i, j)
    
    if start_pos is None or goal_pos is None:
        print("Error: Posisi ambulan atau rumah sakit tidak ditemukan!")
    else:
        print(f"Posisi ambulan: {start_pos}")
        print(f"Posisi rumah sakit: {goal_pos}")
        
        # Jalankan GBFS
        print("\nMencari rute dengan Greedy Best-First Search...")
        gbfs_path, gbfs_visited, gbfs_time = greedy_best_first_search(city_map, start_pos, goal_pos)
        
        if gbfs_path:
            print(f"GBFS berhasil menemukan jalur dengan {len(gbfs_path)} langkah.")
            print(f"Jumlah node yang dikunjungi: {gbfs_visited}")
            print(f"Waktu eksekusi: {gbfs_time:.2f} ms")
        else:
            print("GBFS tidak menemukan jalur!")
        
        # Jalankan A*
        print("\nMencari rute dengan A*...")
        a_star_path, a_star_visited, a_star_time = a_star_search(city_map, start_pos, goal_pos)
        
        if a_star_path:
            print(f"A* berhasil menemukan jalur dengan {len(a_star_path)} langkah.")
            print(f"Jumlah node yang dikunjungi: {a_star_visited}")
            print(f"Waktu eksekusi: {a_star_time:.2f} ms")
        else:
            print("A* tidak menemukan jalur!")
        
        # Visualisasi peta dengan rute
        def visualize_map(city_map, path=None):
            # Buat salinan peta untuk visualisasi
            visual_map = [row[:] for row in city_map]
            
            # Tandai path dengan '+'
            if path:
                for i, j in path:
                    # Jangan menimpa posisi awal dan tujuan
                    if visual_map[i][j] not in ["S", "H"]:
                        visual_map[i][j] = "+"
            
            # Cetak peta
            for row in visual_map:
                print(" ".join(row))
        
        print("\nPeta kota asli:")
        visualize_map(city_map)
        
        print("\nRute GBFS:")
        visualize_map(city_map, gbfs_path)
        
        print("\nRute A*:")
        visualize_map(city_map, a_star_path)