#include "rapp_libraries/NaoSendEmail.h"

#if VMIME_HAVE_SASL_SUPPORT

// SASL authentication handler
class interactiveAuthenticator : public vmime::security::sasl::defaultSASLAuthenticator
{
	const std::vector <vmime::ref <vmime::security::sasl::SASLMechanism> > getAcceptableMechanisms
		(const std::vector <vmime::ref <vmime::security::sasl::SASLMechanism> >& available,
		 vmime::ref <vmime::security::sasl::SASLMechanism> suggested) const
	{
		std::cout << std::endl << "Available SASL mechanisms:" << std::endl;

		for (unsigned int i = 0 ; i < available.size() ; ++i)
		{
			std::cout << "  " << available[i]->getName();

			if (suggested && available[i]->getName() == suggested->getName())
				std::cout << "(suggested)";
		}

		std::cout << std::endl << std::endl;

		return defaultSASLAuthenticator::getAcceptableMechanisms(available, suggested);
	}

	void setSASLMechanism(vmime::ref <vmime::security::sasl::SASLMechanism> mech)
	{
		std::cout << "Trying '" << mech->getName() << "' authentication mechanism" << std::endl;

		defaultSASLAuthenticator::setSASLMechanism(mech);
	}

	const vmime::string getUsername() const
	{
		if (m_username.empty())
			m_username = getUserInput("Username");

		return m_username;
	}

	const vmime::string getPassword() const
	{
		if (m_password.empty())
			m_password = getUserInput("Password");

		return m_password;
	}

	static const vmime::string getUserInput(const std::string& prompt)
	{
		std::cout << prompt << ": ";
		std::cout.flush();

		vmime::string res;
		std::getline(std::cin, res);

		return res;
	}

private:

	mutable vmime::string m_username;
	mutable vmime::string m_password;
};

#else // !VMIME_HAVE_SASL_SUPPORT

// Simple authentication handler
class interactiveAuthenticator : public vmime::security::defaultAuthenticator
{
	const vmime::string getUsername() const
	{
		if (m_username.empty())
			m_username = getUserInput("Username");

		return m_username;
	}

	const vmime::string getPassword() const
	{
		if (m_password.empty())
			m_password = getUserInput("Password");

		return m_password;
	}

	static const vmime::string getUserInput(const std::string& prompt)
	{
		std::cout << prompt << ": ";
		std::cout.flush();

		vmime::string res;
		std::getline(std::cin, res);

		return res;
	}

private:

	mutable vmime::string m_username;
	mutable vmime::string m_password;
};

#endif // VMIME_HAVE_SASL_SUPPORT


#if VMIME_HAVE_TLS_SUPPORT

// Certificate verifier (TLS/SSL)
class interactiveCertificateVerifier : public vmime::security::cert::defaultCertificateVerifier
{
public:

	void verify(vmime::ref <vmime::security::cert::certificateChain> chain)
	{
		try
		{
			setX509TrustedCerts(m_trustedCerts);

			defaultCertificateVerifier::verify(chain);
		}
		catch (vmime::exceptions::certificate_verification_exception&)
		{
			// Obtain subject's certificate
			vmime::ref <vmime::security::cert::certificate> cert = chain->getAt(0);

			//std::cout << std::endl;
			//std::cout << "Server sent a '" << cert->getType() << "'" << " certificate." << std::endl;
			//std::cout << "Do you want to accept this certificate? (Y/n) ";
			//std::cout.flush();

			std::string answer="Y";
			//std::getline(std::cin, answer);

			if (answer.length() != 0 &&
			    (answer[0] == 'Y' || answer[0] == 'y'))
			{
				// Accept it, and remember user's choice for later
				if (cert->getType() == "X.509")
				{
					m_trustedCerts.push_back(cert.dynamicCast
						<vmime::security::cert::X509Certificate>());
				}

				return;
			}

			throw vmime::exceptions::certificate_verification_exception
				("User did not accept the certificate.");
		}
	}

private:

	static std::vector <vmime::ref <vmime::security::cert::X509Certificate> > m_trustedCerts;
};


std::vector <vmime::ref <vmime::security::cert::X509Certificate> >
	interactiveCertificateVerifier::m_trustedCerts;

#endif // VMIME_HAVE_TLS_SUPPORT




//#endif // VMIME_HAVE_TLS_SUPPORT

// Exception helper
static std::ostream& operator<<(std::ostream& os, const vmime::exception& e)
{
	os << "* vmime::exceptions::" << e.name() << std::endl;
	os << "    what = " << e.what() << std::endl;

	// More information for special exceptions
	if (dynamic_cast <const vmime::exceptions::command_error*>(&e))
	{
		const vmime::exceptions::command_error& cee =
			dynamic_cast <const vmime::exceptions::command_error&>(e);

		os << "    command = " << cee.command() << std::endl;
		os << "    response = " << cee.response() << std::endl;
	}

	if (dynamic_cast <const vmime::exceptions::invalid_response*>(&e))
	{
		const vmime::exceptions::invalid_response& ir =
			dynamic_cast <const vmime::exceptions::invalid_response&>(e);

		os << "    response = " << ir.response() << std::endl;
	}

	if (dynamic_cast <const vmime::exceptions::connection_greeting_error*>(&e))
	{
		const vmime::exceptions::connection_greeting_error& cgee =
			dynamic_cast <const vmime::exceptions::connection_greeting_error&>(e);

		os << "    response = " << cgee.response() << std::endl;
	}

	if (dynamic_cast <const vmime::exceptions::authentication_error*>(&e))
	{
		const vmime::exceptions::authentication_error& aee =
			dynamic_cast <const vmime::exceptions::authentication_error&>(e);

		os << "    response = " << aee.response() << std::endl;
	}

	if (dynamic_cast <const vmime::exceptions::filesystem_exception*>(&e))
	{
		const vmime::exceptions::filesystem_exception& fse =
			dynamic_cast <const vmime::exceptions::filesystem_exception&>(e);

		os << "    path = " << vmime::platform::getHandler()->
			getFileSystemFactory()->pathToString(fse.path()) << std::endl;
	}

	if (e.other() != NULL)
		os << *e.other();

	return os;
}

/** Send a message interactively.
  */


void SendEmailClass::sendMessage(std::string login, std::string password, std::string sendTo){
	
	vmime::platform::setHandler<vmime::platforms::posix::posixHandler>();
		try
		{
			vmime::ref <vmime::net::session> sess
	= vmime::create <vmime::net::session>();
			

			// Request user to enter an URL
			vmime::utility::url url("smtp://smtp.gmail.com");

			vmime::ref <vmime::net::transport> tr =
				sess->getTransport(url);

	//#if VMIME_HAVE_TLS_SUPPORT

			// Enable TLS support if available
			std::cout<<"[Sending a message] - Setting properties"<<std::endl;
			tr->setProperty("connection.tls", true);
			tr->setProperty("options.need-authentication", true);
			tr->setProperty("auth.username", login);
			tr->setProperty("auth.password", password);

			// Set the object responsible for verifying certificates, in the
			// case a secured connection is used (TLS/SSL)
			tr->setCertificateVerifier
				(vmime::create <interactiveCertificateVerifier>());

	//#endif // VMIME_HAVE_TLS_SUPPORT

			

			vmime::string fromString;
			fromString = login;
			vmime::mailbox from(fromString);
			vmime::mailboxList to;

			to.appendMailbox(vmime::create <vmime::mailbox>(sendTo));
			
			//std::cout << "Enter message data, including headers (end with '.' on a single line):" << std::endl;
			// Connect to server
			
			tr->connect();
			

			// Send the message

			// Building a message
			std::cout<<"[Sending a message] - Preparing a message content with attachments"<<std::endl;
			vmime::messageBuilder mb;
			mb.setSubject(vmime::text("MessageSubject"));
			mb.setExpeditor(vmime::mailbox(login));
			mb.getRecipients().appendAddress(vmime::create <vmime::mailbox>(sendTo));
			mb.getTextPart()->setText(vmime::create <vmime::stringContentHandler>("I'm writing this short text to test message construction " \
				"with attachment, using the vmime::messageBuilder component."));

			// Attaching Audio and Image file
			vmime::ref<vmime::fileAttachment> attAudio = vmime::create<vmime::fileAttachment>
			(
				AUDIO_PATH,
				vmime::mediaType("application/octet-stream"),
				vmime::text("Sound recorded")
			);

			vmime::ref<vmime::fileAttachment> attImage = vmime::create<vmime::fileAttachment>
			(
				IMAGE_PATH,
				vmime::mediaType("application/octet-stream"),
				vmime::text("Image captured")
			);
			std::cout<< "[Sending a message] - Attaching files"<<std::endl;
			mb.attach(attAudio);
			mb.attach(attImage);

			// Construction
			vmime::ref <vmime::message> msg = mb.construct();

			// Raw text generation
			vmime::string dataToSend = msg->generate();

			vmime::utility::inputStreamStringAdapter is(dataToSend) ;

			tr->send(from, to, is, dataToSend.length());
			std::cout<<"[Sending a message] - Sending a message with attachments"<<std::endl;

			
			tr->disconnect();
			return;
			
		}
		catch (vmime::exception& e)
		{
			std::cerr << std::endl;
			std::cerr << e << std::endl;
			throw;
		}
		catch (std::exception& e)
		{
			std::cerr << std::endl;
			std::cerr << "std::exception: " << e.what() << std::endl;
			throw;
		}
}