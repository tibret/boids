package game

type Game struct {
	Cells      [][]bool
	Buffer     [][]bool
	save       [][]bool
	Size       int32
	Generation int
}

func (g *Game) Clear() {
	for i := range g.Size {
		for k := range g.Size {
			g.Cells[i][k] = false
		}
	}
}

func (g *Game) Save() {
	for i := range g.Size {
		for j := range g.Size {
			g.save[i][j] = g.Cells[i][j]
		}
	}
}

func (g *Game) Load() {
	for i := range g.Size {
		for j := range g.Size {
			g.Cells[i][j] = g.save[i][j]
		}
	}

}

func NewGame(size int) *Game {
	c := make([][]bool, size)
	for i := range size {
		c[i] = make([]bool, size)
	}

	var buffer = make([][]bool, size)
	for i := range size {
		buffer[i] = make([]bool, size)
	}

	var save = make([][]bool, size)
	for i := range size {
		save[i] = make([]bool, size)
	}

	for i := range size {
		for j := range size {
			c[i][j] = false
			buffer[i][j] = false
			save[i][j] = false
		}
	}

	c[61][61] = true
	c[62][61] = true
	c[62][60] = true
	c[62][62] = true
	c[63][62] = true

	return &Game{Cells: c, Generation: 0, Size: int32(size), Buffer: buffer, save: save}
}

func (g *Game) Advance() {
	for i := int32(0); i < g.Size; i++ {
		for j := int32(0); j < g.Size; j++ {
			alive := g.Cells[i][j]
			liveNeighbors := 0

			neighbors := Neighbors(i, j, g.Size)
			for _, n := range neighbors {
				// print("{", n.X, ", ", n.Y, "}, ")
				if g.Cells[n.X][n.Y] {
					liveNeighbors++
				}
			}

			if alive {
				if liveNeighbors < 2 {
					alive = false
				} else if liveNeighbors > 3 {
					alive = false
				}
			} else {
				if liveNeighbors == 3 {
					alive = true
				}
			}

			g.Buffer[i][j] = alive
		}
	}

	for i := int32(0); i < g.Size; i++ {
		for j := int32(0); j < g.Size; j++ {
			g.Cells[i][j] = g.Buffer[i][j]
		}
	}

	g.Generation = g.Generation + 1
}

var Neighbor_Offsets = [8][2]int32{{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}

type Coord struct {
	X int32
	Y int32
}

func Neighbors(i int32, j int32, size int32) []Coord {
	neighbors := make([]Coord, 8)
	for n := range 8 {
		offsets := Neighbor_Offsets[n]
		x := (i + size + offsets[0]) % size
		y := (j + size + offsets[1]) % size

		neighbors[n] = Coord{X: x, Y: y}
	}

	return neighbors
}
