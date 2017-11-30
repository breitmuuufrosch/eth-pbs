#pragma once

namespace pbs17 {
	class Edge {
	public:
		Edge(int a, int b)
			: _a(a), _b(b) {}


		int getA() const {
			return _a;
		}

		int getB() const {
			return _b;
		}

	private:

		int _a;
		int _b;
	};
}