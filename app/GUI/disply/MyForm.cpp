#include "MyForm.h"

using namespace disply;//�O�o�n��.h����namespace�@�˦W�r

[STAThreadAttribute]
int main(array<System::String^>^ args)
{
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	// �إߥD�����ð���
	Application::Run(gcnew MyForm());
	return 0;
}