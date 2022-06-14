#pragma once
#include <iostream>
#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include <GL/freeglut.h>
#include "rs232.h"
#include "UART.h"
#include "Attitude_Kinematics.h"


#define NUM_OF_DATA 3
#define BUF_SIZE 4096

#define PI 3.14159
#define YAW_OFFSET_DEG 95.0

UART_Handler UART;
bool UART_Status = 0;
bool UART_IsUpdated = 0;
int cport_nr;
int baud_rate;
double eul[3], qua[4], R[9];
int alarmStatus = 0;

/* OpenGL*/
GLfloat d = 0;

GLfloat V[8][3] = {
	{-0.5,0.5,0.5},
	{0.5,0.5,0.5},
	{0.5,-0.5,0.5},
	{-0.5,-0.5,0.5},
	{-0.5,0.5,-0.5},
	{0.5,0.5,-0.5},
	{0.5,-0.5,-0.5},
	{-0.5,-0.5,-0.5},
};

GLfloat rV[8][3];
GLfloat L[4][3] = {
	{0,0,0},
	{1,0,0},
	{0,1,0},
	{0,0,1},//改回右手定則顯示
};
GLfloat rL[4][3];



/* Function Declaration */
void Spin();
void DrawCoor(GLfloat L0[], GLfloat L1[], GLfloat L2[], GLfloat L3[]);
void Face(GLfloat A[], GLfloat B[], GLfloat C[], GLfloat D[]);
void Cube(GLfloat V0[], GLfloat V1[], GLfloat V2[], GLfloat V3[], GLfloat V4[], GLfloat V5[], GLfloat V6[], GLfloat V7[]);


namespace disply {
	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO::Ports;
	using namespace System::Runtime::InteropServices;

	//[DllImport("opengl32.dll")]
	//extern HDC wglSwapBuffers(HDC hdc);

	/// <summary>
	/// MyForm 的摘要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			OpenGLInit();
			//
			//TODO:  在此加入建構函式程式碼
			findport();
			/* UART Initialization */
			UART_Status = 0;
		}

		void OpenGLInit(void)
		{
			// Get Handle
			m_hWnd = (HWND)this->OpenGLPanel->Handle.ToInt32();
			m_hDC = GetDC(m_hWnd);
			//WGL::wglSwapBuffers((unsigned int)m_hDC);
			SwapBuffers(m_hDC);

			// Set pixel format
			PIXELFORMATDESCRIPTOR pfd;
			//WGL::ZeroPixelDescriptor(pfd);
			pfd.nVersion = 1;
			pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
			pfd.iPixelType = (byte)(PFD_TYPE_RGBA);
			pfd.cColorBits = 32;
			pfd.cDepthBits = 32;
			pfd.iLayerType = (byte)(PFD_MAIN_PLANE);

			int nFormat = ChoosePixelFormat(m_hDC, &pfd);
			SetPixelFormat(m_hDC, nFormat, &pfd);

			// Create OpenGL Rendering Context
			m_hRC = (wglCreateContext(m_hDC));
			if (m_hRC == 0)
				MessageBox::Show("wglCreateContext Error");
			// Assign OpenGL Rendering Context
			if (wglMakeCurrent(m_hDC, m_hRC) == false)
				MessageBox::Show("wglMakeCurrent Error");

			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LEQUAL);

			glMatrixMode(GL_PROJECTION);//設定觀察點
			glLoadIdentity();
			gluPerspective(75.0f, 1, 1, 1000.0f);

			glMatrixMode(GL_MODELVIEW);//設定觀察點
			glLoadIdentity();
			gluLookAt(-2, 0, 0, 0, 0, 0, 0, 0, 1);//設定觀察點

			timer2->Interval = 10;
			timer2->Enabled = true;
		}
	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::ComboBox^ combox;
	protected:

	private: System::Windows::Forms::ComboBox^ baudbox;

	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::Label^ label2;
	private: System::Windows::Forms::Button^ connect_btn;
	private: System::Windows::Forms::Panel^ OpenGLPanel;
	private: System::IO::Ports::SerialPort^ serialPort1;
	private: System::Windows::Forms::Timer^ timer1;
	private: System::ComponentModel::IContainer^ components;
	private: System::Windows::Forms::PictureBox^ pictureBox1;
	private: System::Windows::Forms::RichTextBox^ showmessage;
	private: System::Windows::Forms::Label^ roll_label;
	private: System::Windows::Forms::Label^ pitch_label;
	private: System::Windows::Forms::Label^ yaw_label;
	private: System::Windows::Forms::GroupBox^ groupBox1;
	private: System::Windows::Forms::TextBox^ roll_box;
	private: System::Windows::Forms::TextBox^ pitch_box;
	private: System::Windows::Forms::TextBox^ yaw_box;
	private: System::Windows::Forms::Timer^ timer2;




	protected:




	protected:

	protected:

	private:
		/// <summary>
		/// 設計工具所需的變數。
		int timetick = 0;
		static HWND m_hWnd;
		static HDC  m_hDC;
		static HGLRC m_hRC;
		
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
		/// 這個方法的內容。
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->combox = (gcnew System::Windows::Forms::ComboBox());
			this->baudbox = (gcnew System::Windows::Forms::ComboBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->connect_btn = (gcnew System::Windows::Forms::Button());
			this->OpenGLPanel = (gcnew System::Windows::Forms::Panel());
			this->serialPort1 = (gcnew System::IO::Ports::SerialPort(this->components));
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->showmessage = (gcnew System::Windows::Forms::RichTextBox());
			this->roll_label = (gcnew System::Windows::Forms::Label());
			this->pitch_label = (gcnew System::Windows::Forms::Label());
			this->yaw_label = (gcnew System::Windows::Forms::Label());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->yaw_box = (gcnew System::Windows::Forms::TextBox());
			this->pitch_box = (gcnew System::Windows::Forms::TextBox());
			this->roll_box = (gcnew System::Windows::Forms::TextBox());
			this->timer2 = (gcnew System::Windows::Forms::Timer(this->components));
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->groupBox1->SuspendLayout();
			this->SuspendLayout();
			// 
			// combox
			// 
			this->combox->FormattingEnabled = true;
			this->combox->Location = System::Drawing::Point(36, 68);
			this->combox->Name = L"combox";
			this->combox->Size = System::Drawing::Size(101, 20);
			this->combox->TabIndex = 1;
			this->combox->Text = L"COM7";
			// 
			// baudbox
			// 
			this->baudbox->FormattingEnabled = true;
			this->baudbox->Items->AddRange(gcnew cli::array< System::Object^  >(2) { L"9600", L"115200" });
			this->baudbox->Location = System::Drawing::Point(161, 68);
			this->baudbox->Name = L"baudbox";
			this->baudbox->Size = System::Drawing::Size(121, 20);
			this->baudbox->TabIndex = 2;
			this->baudbox->Text = L"115200";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(34, 49);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(31, 12);
			this->label1->TabIndex = 3;
			this->label1->Text = L"COM";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(159, 49);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(50, 12);
			this->label2->TabIndex = 3;
			this->label2->Text = L"Baud rate";
			// 
			// connect_btn
			// 
			this->connect_btn->Location = System::Drawing::Point(306, 49);
			this->connect_btn->Name = L"connect_btn";
			this->connect_btn->Size = System::Drawing::Size(75, 39);
			this->connect_btn->TabIndex = 4;
			this->connect_btn->Text = L"Connect";
			this->connect_btn->UseVisualStyleBackColor = true;
			this->connect_btn->Click += gcnew System::EventHandler(this, &MyForm::connect_btn_Click);
			// 
			// OpenGLPanel
			// 
			this->OpenGLPanel->BackColor = System::Drawing::SystemColors::ControlDark;
			this->OpenGLPanel->Location = System::Drawing::Point(36, 105);
			this->OpenGLPanel->Name = L"OpenGLPanel";
			this->OpenGLPanel->Size = System::Drawing::Size(370, 268);
			this->OpenGLPanel->TabIndex = 7;
			// 
			// timer1
			// 
			this->timer1->Interval = 10;
			this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
			// 
			// pictureBox1
			// 
			this->pictureBox1->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pictureBox1->Location = System::Drawing::Point(387, 68);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(19, 20);
			this->pictureBox1->TabIndex = 9;
			this->pictureBox1->TabStop = false;
			// 
			// showmessage
			// 
			this->showmessage->Location = System::Drawing::Point(440, 154);
			this->showmessage->Name = L"showmessage";
			this->showmessage->Size = System::Drawing::Size(287, 219);
			this->showmessage->TabIndex = 10;
			this->showmessage->Text = L"";
			// 
			// roll_label
			// 
			this->roll_label->AutoSize = true;
			this->roll_label->Location = System::Drawing::Point(17, 30);
			this->roll_label->Name = L"roll_label";
			this->roll_label->Size = System::Drawing::Size(25, 12);
			this->roll_label->TabIndex = 11;
			this->roll_label->Text = L"Roll";
			// 
			// pitch_label
			// 
			this->pitch_label->AutoSize = true;
			this->pitch_label->Location = System::Drawing::Point(115, 30);
			this->pitch_label->Name = L"pitch_label";
			this->pitch_label->Size = System::Drawing::Size(28, 12);
			this->pitch_label->TabIndex = 11;
			this->pitch_label->Text = L"Pitch";
			// 
			// yaw_label
			// 
			this->yaw_label->AutoSize = true;
			this->yaw_label->Location = System::Drawing::Point(216, 30);
			this->yaw_label->Name = L"yaw_label";
			this->yaw_label->Size = System::Drawing::Size(26, 12);
			this->yaw_label->TabIndex = 11;
			this->yaw_label->Text = L"Yaw";
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->yaw_box);
			this->groupBox1->Controls->Add(this->pitch_box);
			this->groupBox1->Controls->Add(this->roll_box);
			this->groupBox1->Controls->Add(this->roll_label);
			this->groupBox1->Controls->Add(this->pitch_label);
			this->groupBox1->Controls->Add(this->yaw_label);
			this->groupBox1->Location = System::Drawing::Point(440, 49);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(287, 87);
			this->groupBox1->TabIndex = 13;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Attitude (deg.)";
			// 
			// yaw_box
			// 
			this->yaw_box->BackColor = System::Drawing::SystemColors::ActiveBorder;
			this->yaw_box->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->yaw_box->Location = System::Drawing::Point(218, 45);
			this->yaw_box->Name = L"yaw_box";
			this->yaw_box->Size = System::Drawing::Size(51, 22);
			this->yaw_box->TabIndex = 14;
			// 
			// pitch_box
			// 
			this->pitch_box->BackColor = System::Drawing::SystemColors::ActiveBorder;
			this->pitch_box->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->pitch_box->Location = System::Drawing::Point(117, 45);
			this->pitch_box->Name = L"pitch_box";
			this->pitch_box->Size = System::Drawing::Size(51, 22);
			this->pitch_box->TabIndex = 14;
			// 
			// roll_box
			// 
			this->roll_box->BackColor = System::Drawing::SystemColors::ActiveBorder;
			this->roll_box->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->roll_box->Location = System::Drawing::Point(19, 45);
			this->roll_box->Name = L"roll_box";
			this->roll_box->Size = System::Drawing::Size(51, 22);
			this->roll_box->TabIndex = 14;
			// 
			// timer2
			// 
			this->timer2->Tick += gcnew System::EventHandler(this, &MyForm::timer2_Tick);
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(797, 403);
			this->Controls->Add(this->showmessage);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->OpenGLPanel);
			this->Controls->Add(this->connect_btn);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->baudbox);
			this->Controls->Add(this->combox);
			this->Controls->Add(this->groupBox1);
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: void findport() {
		array<Object^>^ objectArray = SerialPort::GetPortNames();
		this->combox->Items->AddRange(objectArray);
	}
	private: System::Void connect_btn_Click(System::Object^ sender, System::EventArgs^ e)
	{
		timer1->Enabled = true;

		/* UART Initialization */
		for (int i = 1; i < 40; i++)
		{
			if (combox->Text == ("COM" + i))
			{
				/*Console::WriteLine("Hello\r\n");*/
				cport_nr = i;
				break;
			}
		}
		baud_rate = Convert::ToInt32(baudbox->Text); // Baud rate of UART
		
		/* UART Initialization */
		if (UART_Status == 0)
		{
			if (UART.Init(cport_nr, baud_rate) < 0)
			{
				showmessage->AppendText("Can not open comport " + (cport_nr) + " ... \r");
			}
			else
			{
				UART_Status = 1;
				pictureBox1->BackColor = System::Drawing::Color::LightPink;
				combox->Enabled = false;
				baudbox->Enabled = false;

				showmessage->AppendText("Com Port " + (cport_nr) + " has been opened successfully ...\r");
			}
		}
		else
		{
			UART_Status = 0;
			UART_IsUpdated = 0;
			UART.Close();

			pictureBox1->BackColor = System::Drawing::SystemColors::ActiveCaption;
			combox->Enabled = true;
			baudbox->Enabled = true;

			showmessage->AppendText("Com Port " + (cport_nr) + " has been closed ...\r");
		}
	}
	private: System::Void timer1_Tick(System::Object^ sender, System::EventArgs^ e)
	{
		//String^ data = serialPort1->ReadExisting();
		//String^ data= serialPort1->ReadLine();
		//Console::WriteLine(data);
		static int timer_count = 0;
		int n;

		/* Receiving and Decoding the Serial Binary Data */
		if (UART_Status == 1)
		{
			UART_IsUpdated = UART.Receive(&n);
			if (UART_IsUpdated == 1)
			{ 
				memcpy(qua, UART.RxData.qua, sizeof(qua));
				alarmStatus = UART.RxData.status;
				qua2eul(qua, eul);
				qua2dcm(qua, R);

				roll_box->Text = (Convert::ToString((double)((int)(eul[0] * RAD_TO_DEG * 100) / 100.0)));
				pitch_box->Text = (Convert::ToString((double)((int)(eul[1] * RAD_TO_DEG  * 100) / 100.0)));
				yaw_box->Text = (Convert::ToString((double)((int)(eul[2] * RAD_TO_DEG * 100) / 100.0)));
			} /* End if (UART_Status == 1) */
			else
			{
				showmessage->AppendText("UART decoding failed qaq\r\n");
			}
		}
		
		/* Print Infomation to USER */
		if (timer_count++ >= 100)
		{
			timer_count = 0;

			if (UART_IsUpdated == 1)
			{
				showmessage->AppendText("received " + n + " bytes \r\n");
			}
			else
			{
				
			}

		}
	} /* End of timer1_Tick() */



	private: System::Void timer2_Tick(System::Object^ sender, System::EventArgs^ e)
	{
		wglMakeCurrent(m_hDC, m_hRC);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		
		Spin();
		glFlush();
		SwapBuffers(m_hDC);
	}
}; /* End of class MyForm */
}

void Spin()
{
	//double eul[3], R[9];
	//for (int i = 0; i < 3; i++)
	//{
	//	eul[i] = UART.RxData.Value[i]/180.0*PI;
	//}
	//eul[2] = eul[2] - YAW_OFFSET_DEG/180.0*PI;
	//eul2dcm(eul, R);

	// Rotate cube
	double Vb[3];
	for (int i = 0; i < 8; i++)
	{
		/*Vb[0] = V[i][0];
		Vb[1] = -V[i][2];
		Vb[2] = V[i][1];*/

		Vb[0] = V[i][0];
		Vb[1] = V[i][1];
		Vb[2] = V[i][2];
		 
		rV[i][0] = R[0] * Vb[0] + R[1] * Vb[1] + R[2] * Vb[2];
		rV[i][1] = R[3] * Vb[0] + R[4] * Vb[1] + R[5] * Vb[2];
		rV[i][2] = R[6] * Vb[0] + R[7] * Vb[1] + R[8] * Vb[2];
		//rV[i][0] = R[0] * Vb[0] + R[3] * Vb[1] + R[6] * Vb[2];
		//rV[i][1] = R[1] * Vb[0] + R[4] * Vb[1] + R[7] * Vb[2];
		//rV[i][2] = R[2] * Vb[0] + R[5] * Vb[1] + R[8] * Vb[2];
	}

	// Rotate coor.
	for (int i = 0; i < 4; i++)
	{
		rL[i][0] = R[0] * L[i][0] + R[1] * L[i][1] + R[2] * L[i][2];
		rL[i][1] = R[3] * L[i][0] + R[4] * L[i][1] + R[5] * L[i][2];
		rL[i][2] = R[6] * L[i][0] + R[7] * L[i][1] + R[8] * L[i][2];
		//rL[i][0] = R[0] * L[i][0] + R[3] * L[i][1] + R[6] * L[i][2];
		//rL[i][1] = R[1] * L[i][0] + R[4] * L[i][1] + R[7] * L[i][2];
		//rL[i][2] = R[2] * L[i][0] + R[5] * L[i][1] + R[8] * L[i][2];
	}
	
	Cube(rV[0], rV[1], rV[2], rV[3], rV[4], rV[5], rV[6], rV[7]);
	DrawCoor(rL[0], rL[1], rL[2], rL[3]);

}

void Face(GLfloat A[], GLfloat B[], GLfloat C[], GLfloat D[])
{
	glBegin(GL_POLYGON);
	glVertex3fv(A);
	glVertex3fv(B);
	glVertex3fv(C);
	glVertex3fv(D);
	glEnd();
}

void Cube(GLfloat V0[], GLfloat V1[], GLfloat V2[], GLfloat V3[], GLfloat V4[], GLfloat V5[], GLfloat V6[], GLfloat V7[])
{
	GLfloat alpha = 0.5;
	GLfloat a = 255.0f;
	GLfloat color_x[3] = { 255/ a , 192/ a, 203 / a }; //粉紅色
	GLfloat color_y[3] = { 255/ a, 227 / a, 132 / a }; //黃色
	GLfloat color_z[3] = { 135/ a , 206 / a, 235 / a };//天藍色
	glColor4f(color_y[0], color_y[1], color_y[2], alpha);
	Face(V0, V1, V2, V3); //front
	glColor4f(color_y[0], color_y[1], color_y[2], alpha);
	Face(V4, V5, V6, V7);//back
	glColor4f(color_x[0], color_x[1], color_x[2], alpha);
	Face(V0, V4, V7, V3); //left
	glColor4f(color_x[0], color_x[1], color_x[2], alpha);
	Face(V1, V5, V6, V2);//right
	glColor4f(color_z[0], color_z[1], color_z[2], alpha);
	Face(V2, V3, V7, V6); //bot
	glColor4f(color_z[0], color_z[1], color_z[2], alpha);
	Face(V0, V1, V5, V4);//top
}

void DrawCoor(GLfloat L0[], GLfloat L1[], GLfloat L2[], GLfloat L3[])
{
	glLineWidth(3.0f);

	glColor3f(1.0f, 0.0f, 0.0f); //畫紅色的x軸
	glBegin(GL_LINES);
	glVertex3fv(L0);
	glVertex3fv(L1);
	glEnd();

	glColor3f(0.0, 1.0, 0.0); //畫綠色的y軸
	glBegin(GL_LINES);
	glVertex3fv(L0);
	glVertex3fv(L2);
	glEnd(); 

	glColor3f(0.0, 0.0, 1.0); //畫藍色的z軸
	glBegin(GL_LINES);
	glVertex3fv(L0);
	glVertex3fv(L3);
	glEnd();

}
