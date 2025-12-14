using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using System.Web.Script.Serialization;
using System.Windows.Forms;

namespace WindowsFormsApplication1
{
    public partial class Form1 : Form
    {
        UdpClient ctrludpcRecv = null;  //控制指令
        IPEndPoint ctrllocalIpep = null;

        public Form1()
        {
            InitializeComponent();

            ctrllocalIpep = new IPEndPoint(IPAddress.Any, 8898); // 本机IP和监听端口号
            ctrludpcRecv = new UdpClient(ctrllocalIpep);

            ctrludpcRecv.Client.ReceiveBufferSize = 64 * 4096 * 4096;
            ctrludpcRecv.EnableBroadcast = true;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            IPAddress Adrr;
            if (string.IsNullOrEmpty(textBox1.Text))
            {
                Adrr = IPAddress.Parse("255.255.255.255");
            }
            else
            {
                Adrr = IPAddress.Parse(textBox1.Text);
            }
            

            IPEndPoint remoteIpep = new IPEndPoint(Adrr, 50001); ;

            JavaScriptSerializer ser = new JavaScriptSerializer();

            CtrlJson rcj = new CtrlJson() { cmd = "forward" };
            string outputJson = ser.Serialize(rcj);
            byte[] sendByte = System.Text.Encoding.Default.GetBytes(outputJson);

            //发送
            ctrludpcRecv.Send(sendByte, sendByte.Length, remoteIpep);
        }

        private void button2_Click(object sender, EventArgs e)
        {
            IPAddress Adrr;
            if (string.IsNullOrEmpty(textBox1.Text))
            {
                Adrr = IPAddress.Parse("255.255.255.255");
            }
            else
            {
                Adrr = IPAddress.Parse(textBox1.Text);
            }

            IPEndPoint remoteIpep = new IPEndPoint(Adrr, 50001); ;

            JavaScriptSerializer ser = new JavaScriptSerializer();

            CtrlJson rcj = new CtrlJson() { cmd = "backward" };
            string outputJson = ser.Serialize(rcj);
            byte[] sendByte = System.Text.Encoding.Default.GetBytes(outputJson);

            //发送
            ctrludpcRecv.Send(sendByte, sendByte.Length, remoteIpep);
        }

        private void button4_Click(object sender, EventArgs e)
        {
            IPAddress Adrr;
            if (string.IsNullOrEmpty(textBox1.Text))
            {
                Adrr = IPAddress.Parse("255.255.255.255");
            }
            else
            {
                Adrr = IPAddress.Parse(textBox1.Text);
            }

            IPEndPoint remoteIpep = new IPEndPoint(Adrr, 50001); ;

            JavaScriptSerializer ser = new JavaScriptSerializer();

            CtrlJson rcj = new CtrlJson() { cmd = "right" };
            string outputJson = ser.Serialize(rcj);
            byte[] sendByte = System.Text.Encoding.Default.GetBytes(outputJson);

            //发送
            ctrludpcRecv.Send(sendByte, sendByte.Length, remoteIpep);
        }

        private void button3_Click(object sender, EventArgs e)
        {
            IPAddress Adrr;
            if (string.IsNullOrEmpty(textBox1.Text))
            {
                Adrr = IPAddress.Parse("255.255.255.255");
            }
            else
            {
                Adrr = IPAddress.Parse(textBox1.Text);
            }

            IPEndPoint remoteIpep = new IPEndPoint(Adrr, 50001); ;

            JavaScriptSerializer ser = new JavaScriptSerializer();

            CtrlJson rcj = new CtrlJson() { cmd = "left" };
            string outputJson = ser.Serialize(rcj);
            byte[] sendByte = System.Text.Encoding.Default.GetBytes(outputJson);

            //发送
            ctrludpcRecv.Send(sendByte, sendByte.Length, remoteIpep);
        }
    }
}
