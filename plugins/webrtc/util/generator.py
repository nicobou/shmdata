from OpenSSL import crypto, SSL


class Generator:
    """
    Utility class for creating an X509 certificate
    and its key
    Based on this: https://stackoverflow.com/questions/27164354/create-a-self-signed-x509-certificate-in-python
    The corresponding command line to generate this certificate is:
    openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
    """
    def __init__(
            self,
            name="name",
            email="email",
            country="country", # should be a 2 letters abbreviation
            state_province="state_or_province",
            locality="locality",
            organization="organization",
            organization_unit="organization_unit",
            serial_number=0,
            validity_start_in_seconds=0,
            validity_end_in_seconds=12 * 365 * 24 * 60 * 60,
            key_file="key.pem",
            cert_file="cert.pem"
            ) -> None:
        # Create a key pair
        self.key_pair = crypto.PKey()
        self.key_pair.generate_key(crypto.TYPE_RSA, 4096)

	# Create a self-signed cert
        self.cert = crypto.X509()
        self.cert.get_subject().CN = name
        self.cert.get_subject().emailAddress = email
        # Set the country only if input valid
        if len(country) == 2:
            self.cert.get_subject().C = country
        self.cert.get_subject().ST = state_province
        self.cert.get_subject().L = locality
        self.cert.get_subject().O = organization
        self.cert.get_subject().OU = organization_unit
        self.cert.set_serial_number(serial_number)
        self.cert.gmtime_adj_notBefore(validity_start_in_seconds)
        self.cert.gmtime_adj_notAfter(validity_end_in_seconds)
        self.cert.set_issuer(self.cert.get_subject())
        self.cert.set_pubkey(self.key_pair)
        self.cert.sign(self.key_pair, "sha512")

    def generate_certificate(self, key_file="key.pem",cert_file="cert.pem") -> None:
        with open(cert_file, "wt") as f:
            f.write(crypto.dump_certificate(crypto.FILETYPE_PEM, self.cert).decode("utf-8"))
        with open(key_file, "wt") as f:
            f.write(crypto.dump_privatekey(crypto.FILETYPE_PEM, self.key_pair).decode("utf-8"))


if __name__ == "__main__":
    cert = Generator()
    cert.generate_certificate()
