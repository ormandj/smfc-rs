FROM rust:1.94.0-slim-bookworm AS builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends musl-tools \
    && rm -rf /var/lib/apt/lists/*
RUN rustup target add x86_64-unknown-linux-musl
WORKDIR /build
COPY Cargo.toml Cargo.lock ./
COPY src/ src/
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/usr/local/cargo/git \
    --mount=type=cache,target=/build/target \
    cargo build --release --target x86_64-unknown-linux-musl \
    && cp target/x86_64-unknown-linux-musl/release/smfc-rs /smfc-rs

FROM scratch
COPY --from=builder /smfc-rs /smfc-rs
ENTRYPOINT ["/smfc-rs"]
