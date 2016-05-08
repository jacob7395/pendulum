
#include "UDP_RxTx.h"

int UDP(void)
{
   int sockfd,n;
   struct sockaddr_in servaddr,cliaddr;
   socklen_t len;
   char mesg[1000];
   int MessageLength;

	// create a UDP socket
   if ((sockfd=socket(AF_INET,SOCK_DGRAM,0))==-1) {
	   printf("socket() failed\n");
	   exit(0);
	} else;

   bzero(&servaddr,sizeof(servaddr));			// clear servaddr structure
   servaddr.sin_family = AF_INET;				// use IPV4 addresses
   servaddr.sin_addr.s_addr=htonl(INADDR_ANY);	// allow any IP address to communicate
   servaddr.sin_port=htons(3490);				// the port we will use on this computer
   // bind socket to above
   if (bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr))==-1) {
	   printf("bind() failed\n");
	   exit(0);
	} else;

	// wait for message from any IP address sent to port 3490
	// the source IP address wil be in cliaddr after the message has been rx
	// send a message to cliaddr

   for (;;)
   {
      len = sizeof(cliaddr);
      n = recvfrom(sockfd,mesg,1000,0,(struct sockaddr *)&cliaddr,&len);
      printf("-------------------------------------------------------\n");
      mesg[n] = 0;
      printf("Received the following:\n");
      printf("%s",mesg);
      printf("-------------------------------------------------------\n");

		strcpy(mesg, "Hello from Pi");
		MessageLength=strlen(mesg);

      n = sendto(sockfd,mesg,MessageLength,0,(struct sockaddr *)&cliaddr,len);

   }
   return(0);
}
