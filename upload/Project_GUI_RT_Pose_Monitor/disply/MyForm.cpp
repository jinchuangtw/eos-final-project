#include "MyForm.h"

using namespace disply;//記得要跟.h中的namespace一樣名字

[STAThreadAttribute]
int main(array<System::String^>^ args)
{
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	// 建立主視窗並執行
	Application::Run(gcnew MyForm());
	return 0;
}