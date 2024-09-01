
#include <iostream>
namespace myspace {
	class A
	{
	public:
		~A() { std::cout << "A deleted!" << std::endl; };
		A() = default;
		A(const int& yy, const int& gg) :x(yy), y(gg) {};
		std::unique_ptr<int> p1;
		void swap()
		{
			int temp;
			temp = x;
			x = y;
			y = temp;
		}
	public:
		int x;
		int y;
	};

	int main()
	{
		std::unique_ptr<A> ptr = std::make_unique<A>(12, 13);
		ptr->swap();
		std::cout << ptr->x << " " << ptr->y << std::endl;
		system("pause");
		return 0;
	}
}  //myspace